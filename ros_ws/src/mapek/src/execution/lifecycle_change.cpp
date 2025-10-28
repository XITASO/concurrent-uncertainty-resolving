#include <mapek/execution/lifecycle_change.hpp>
#include "mapek/util/logger.hpp"
#include <system_interfaces/msg/experiment_logging.hpp>
#include <system_interfaces/msg/strategy_status.hpp>

using AdaptationType = system_interfaces::msg::AdaptationType;
using AdaptationStatus = system_interfaces::msg::AdaptationStatus;
LifecycleChangeServiceHandler::LifecycleChangeServiceHandler(rclcpp::Node::SharedPtr node, const std::string &component_name)
: node_(node)
{
    // Initialize the service client
    std::string service_name = "/" + string_utils::strip(component_name, '/') + "/set_lifecycle_changes";
    _component_name = component_name;
    client_ = node_->create_client<system_interfaces::srv::SetLifecycleChanges>(service_name);
    relevantAdaptationTypes = {AdaptationType::ACTION_ACTIVATE, AdaptationType::ACTION_DEACTIVATE, AdaptationType::ACTION_RESTART};
}

bool LifecycleChangeServiceHandler::setupRequest(AdaptationMap&adaptations, MAPEK_Graph &graph)
{
    transitions.clear();
    finished_adaptations.clear();

    if (adaptations.find(_component_name) == adaptations.end())
        return false;
    auto splitAdaptations = adaptations_utils::splitAdaptations(adaptations[_component_name], relevantAdaptationTypes);
    std::shared_ptr<BTLogger> logger = BTLogger::get_global_logger();
    bool triggeredAdaptations = false;

    if (splitAdaptations.first.size() > 0)
    {
        triggeredAdaptations = true;
        current_adaptations.push_back(splitAdaptations.first[0]);
        auto lc_change = adaptations_utils::toLifecyleAdaptation(splitAdaptations.first[0]);

        transitions.insert(transitions.begin(), lc_change.begin(), lc_change.end());
        graph[_component_name].busy(true);

        auto msg = new system_interfaces::msg::ExperimentLogging();
        msg->timestamp = logger->get_time(); 
        msg->source = "execution_lifecycle_change";  
        msg->adaptation_status = AdaptationStatus::STATUS_ADAPTATION_TRIGGERED;
        logger->silent_global(msg);
        adaptations[_component_name] = splitAdaptations.second;
    }

    return triggeredAdaptations;
}

VectorGenericAdaptations LifecycleChangeServiceHandler::getFailedAdaptations()
{
    return failed_adaptations;
}

VectorGenericAdaptations LifecycleChangeServiceHandler::getFinishedAdaptations()
{
    return finished_adaptations;
}

ServiceStatus LifecycleChangeServiceHandler::tick(MAPEK_Graph &graph)
{
    if (!future_result_.valid() || future_result_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
        auto request = std::make_shared<system_interfaces::srv::SetLifecycleChanges::Request>();
        request->transitions = transitions;
        future_result_ = client_->async_send_request(request);
        return ServiceStatus::RUNNING;
    }
    rclcpp::spin_some(node_);

    if (future_result_.valid() && future_result_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
        graph[_component_name].busy(false);
        auto response = future_result_.get();

        std::shared_ptr<BTLogger> logger = BTLogger::get_global_logger();
        auto msg = new system_interfaces::msg::ExperimentLogging();
        msg->timestamp = logger->get_time(); 
        msg->source = "execution_lifecycle_change" ;  
        msg->adaptation_status = AdaptationStatus::STATUS_ADAPTATION_FINISHED;

        if (parseLifecycleResponse(*response))
        {
            // As long as we do not consider failed adaptations, this will be fine
            finished_adaptations = current_adaptations;
            msg->success = true;
            logger->silent_global(msg);
            return ServiceStatus::SUCCESS;
        }
        logger->silent_global(msg);
        return ServiceStatus::FAILURE;
    }

    return ServiceStatus::RUNNING;
}

bool LifecycleChangeServiceHandler::parseLifecycleResponse(const system_interfaces::srv::SetLifecycleChanges_Response &response)
{
    failed_adaptations.clear();
    int idx = 0;
    for (const auto &result : response.results)
    {
        if (!result)
        {
            failed_adaptations.push_back(current_adaptations[idx]);
        }
        idx++;
    }
    return failed_adaptations.size() == 0;
}