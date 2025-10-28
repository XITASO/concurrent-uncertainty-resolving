#include "bt_mape_k/execution.hpp"
using AdaptationStatus = system_interfaces::msg::AdaptationStatus;
using AdaptationType = system_interfaces::msg::AdaptationType;

BTExecuteModeChange::BTExecuteModeChange(const std::string &name, const BT::NodeConfiguration &config)
: BT::StatefulActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("execute_mode_change_action_node");
    std::string component_name = getInput<std::string>("component_name").value();
    service_handler_ = std::make_unique<ModeChangeServiceHandler>(node_, component_name);
}

BT::NodeStatus BTExecuteModeChange::onStart()
{
    auto bb_adaptations = getInput<AdaptationMap>("adaptations");
    MAPEK_Graph graph = getInput<MAPEK_Graph>("system_state_graph").value();
    AdaptationMap adaptations = bb_adaptations.value();
    bool triggered_adaptation = service_handler_->setupRequest(adaptations, graph);
    if (!triggered_adaptation)
    {
        return BT::NodeStatus::SUCCESS;
    }
    setOutput("adaptations", adaptations);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    switch (service_handler_->tick(graph))
    {
    case ServiceStatus::SUCCESS:
        status = BT::NodeStatus::SUCCESS;
        break;

    case ServiceStatus::FAILURE:
        status = BT::NodeStatus::FAILURE;
        break;
    }

    setOutput("system_state_graph", graph);
    return status;
}

BT::NodeStatus BTExecuteModeChange::onRunning()
{
    MAPEK_Graph graph = getInput<MAPEK_Graph>("system_state_graph").value();
    switch (service_handler_->tick(graph))
    {
    case ServiceStatus::SUCCESS:
    {
        //auto bb_finished_adaptations = getInput<VectorGenericAdaptations>("finished_adaptations").value();
        //auto finished_adaptations = service_handler_->getFinishedAdaptations();
        //bb_finished_adaptations.insert(bb_finished_adaptations.end(), finished_adaptations.begin(), finished_adaptations.end());
        setOutput("system_state_graph", graph);
        //setOutput("finished_adaptations", bb_finished_adaptations);
    }
        return BT::NodeStatus::SUCCESS;

    case ServiceStatus::FAILURE:
    {
        setOutput("system_state_graph", graph);
        setOutput("failed_adaptations", service_handler_->getFailedAdaptations());
    }
        return BT::NodeStatus::FAILURE;

    default:
        return BT::NodeStatus::RUNNING;
    }
}

void BTExecuteModeChange::onHalted() { }

BT::PortsList BTExecuteModeChange::providedPorts()
{
    return {BT::InputPort<std::string>("component_name"),
            BidirectionalPort<AdaptationMap>("adaptations"),
            BidirectionalPort<MAPEK_Graph>("system_state_graph"),
            BidirectionalPort<AdaptationMap>("failed_adaptations")};
}
