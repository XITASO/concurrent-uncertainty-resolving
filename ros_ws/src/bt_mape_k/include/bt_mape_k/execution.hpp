#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_ros2/bt_service_node.hpp>

#include "system_interfaces/msg/comm_change_input.hpp"
#include "system_interfaces/msg/parametrization_input.hpp"
#include "system_interfaces/srv/set_lifecycle_changes.hpp"
#include <system_interfaces/msg/generic_adaptation.hpp>
#include <system_interfaces/msg/adaptation_type.hpp>
#include <system_interfaces/msg/adaptation_status.hpp>
#include <system_interfaces/msg/experiment_logging.hpp>

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"


#include <cstdlib> 
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>

#include <mapek/util/graph.hpp>
#include "mapek/util/adaptation_utils.hpp"
#include "mapek/util/logger.hpp"
#include "mapek/util/string_utils.hpp"

#include <mapek/execution/parametrization.hpp>
#include <mapek/execution/comm_change.hpp>
#include <mapek/execution/lifecycle_change.hpp>
#include <mapek/execution/mode_change.hpp>
#include <mapek/execution/redeploy.hpp>
#include <mapek/util/graph.hpp>
#include <mapek/rules/Adaptation.hpp>

using namespace BT;
using AdaptationType = system_interfaces::msg::AdaptationType;
using VectorGenericAdaptations = std::vector<system_interfaces::msg::GenericAdaptation>;
using AdaptationMap = std::map<std::string, std::vector<GenericAdaptation>>;

class BTExecuteParametrization: public BT::StatefulActionNode
{
public:
    BTExecuteParametrization(const std::string &name, const BT::NodeConfiguration &config);
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<ParametrizationHandler> service_handler_;
};

class BTExecuteModeChange: public BT::StatefulActionNode
{
public:
    BTExecuteModeChange(const std::string &name, const BT::NodeConfiguration &config);
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<ModeChangeServiceHandler> service_handler_;
};

class BTExecuteRedeploy: public BT::StatefulActionNode
{
public:
    BTExecuteRedeploy(const std::string &name, const BT::NodeConfiguration &config);
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<RedeployServiceHandler> service_handler_;
};


class BTExecuteCommChange : public BT::StatefulActionNode
{
public:
    BTExecuteCommChange(const std::string &name, const BT::NodeConfiguration &config);
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<CommChangeServiceHandler> service_handler_;
};

class BTExecuteLifeCycle: public BT::StatefulActionNode
{
public:
    BTExecuteLifeCycle(const std::string &name, const BT::NodeConfiguration &config);
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<LifecycleChangeServiceHandler> service_handler_;
    std::string component_name;
};
