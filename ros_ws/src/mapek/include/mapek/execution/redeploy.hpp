#ifndef REDEPLOY_HANDLER_HPP
#define REDEPLOY_HANDLER_HPP

#include <rclcpp/rclcpp.hpp>

#include <lifecycle_msgs/srv/change_state.hpp>

#include <system_interfaces/srv/set_lifecycle_changes.hpp>
#include <system_interfaces/msg/adaptation_status.hpp>
#include <system_interfaces/msg/adaptation_type.hpp>

#include <mapek/util/definitions.hpp>
#include <mapek/util/adaptation_utils.hpp>
#include <mapek/util/string_utils.hpp>
#include <mapek/util/graph.hpp>

class RedeployServiceHandler
{
public:
    RedeployServiceHandler(rclcpp::Node::SharedPtr node, const std::string &component_name);

    bool setupRequest(AdaptationMap&adaptations, MAPEK_Graph &graph);
    VectorGenericAdaptations getFailedAdaptations();
    VectorGenericAdaptations getFinishedAdaptations();
    ServiceStatus tick(MAPEK_Graph &graph);

private:
    rclcpp::Node::SharedPtr node_;
    std::vector<uint8_t> relevantAdaptationTypes;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_;
    GenericAdaptation adaptation;
    VectorGenericAdaptations failed_adaptations {};
    VectorGenericAdaptations finished_adaptations {};
    std::string executable_name;
    std::string package_name;
    std::string _component_name;
     
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future_result_;
    lifecycle_msgs::msg::Transition transition;
    bool restart_ros2_node(const std::string &package, const std::string &executable);

    bool parseLifecycleResponse(const lifecycle_msgs::srv::ChangeState_Response &response);
};

#endif //REDEPLOY_SERVICE_HANDLER_HPP