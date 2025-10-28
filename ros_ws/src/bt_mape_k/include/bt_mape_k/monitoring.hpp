#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "system_interfaces/msg/set_blackboard_group.hpp"
#include <system_interfaces/msg/set_blackboard.hpp>
#include <mapek/util/graph.hpp>
#include <mapek/monitoring.hpp>


using namespace BT;


class BTMonitoring: public RosTopicSubNode<system_interfaces::msg::SetBlackboardGroup> 
{
public:
    BTMonitoring(const std::string& name, const NodeConfig& conf,
                const RosNodeParams& params)
        : RosTopicSubNode<system_interfaces::msg::SetBlackboardGroup>(name, conf, params)//, blackboard_(conf.blackboard)
        {
            node_handle = params.nh.lock();
            if (!node_handle) // Check if the weak pointer was successfully locked
            {
                throw RuntimeError("[Monitoring] Could not lock the node handle, will not be able to provide the time");
            }
            mapek_monitoring = std::make_unique<Monitoring>();
            setOutput("system_state_graph", mapek_monitoring->get_initial_graph());
        }

    NodeStatus onTick(const std::shared_ptr<system_interfaces::msg::SetBlackboardGroup>& msg) override;
    

    static PortsList providedPorts()
    {
        return {
            OutputPort<int>("clock"),
            BidirectionalPort<ValueStorePtr>("value_store"),
            BidirectionalPort<MAPEK_Graph>("system_state_graph")
        };
    }

private:
    //Blackboard::Ptr blackboard_;
    std::unique_ptr<Monitoring> mapek_monitoring;


    std::shared_ptr<rclcpp::Node> node_handle;
};