#include <mapek/execution/mode_change.hpp>
#include "mapek/util/logger.hpp"
#include <system_interfaces/msg/experiment_logging.hpp>
#include <system_interfaces/msg/strategy_status.hpp>

using AdaptationType = system_interfaces::msg::AdaptationType;
using AdaptationStatus = system_interfaces::msg::AdaptationStatus;
ModeChangeServiceHandler::ModeChangeServiceHandler(rclcpp::Node::SharedPtr node, const std::string &component_name)
    : node_(node)
{
    // Initialize the service client
    std::string service_name = "/" + string_utils::strip(component_name, '/') + "/change_mode";
    _component_name = component_name;
    client_ = node_->create_client<system_modes_msgs::srv::ChangeMode>(service_name);
    relevantAdaptationTypes = {AdaptationType::ACTION_CHANGE_MODE};
}

bool ModeChangeServiceHandler::setupRequest(AdaptationMap &adaptations, MAPEK_Graph &graph)
{
    changes.clear();
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
        mode_msg = adaptations_utils::toSystemMode(splitAdaptations.first[0]);

        graph[_component_name].busy(true);

        auto msg = new system_interfaces::msg::ExperimentLogging();
        msg->timestamp = logger->get_time();
        msg->source = "execution_mode_change";
        msg->adaptation_type = AdaptationType::ACTION_CHANGE_MODE;
        msg->adaptation_status = AdaptationStatus::STATUS_ADAPTATION_TRIGGERED;
        logger->silent_global(msg);
        adaptations[_component_name] = splitAdaptations.second;
    }

    return triggeredAdaptations;
}

VectorGenericAdaptations ModeChangeServiceHandler::getFailedAdaptations()
{
    return failed_adaptations;
}

VectorGenericAdaptations ModeChangeServiceHandler::getFinishedAdaptations()
{
    return finished_adaptations;
}

ServiceStatus ModeChangeServiceHandler::tick(MAPEK_Graph &graph)
{
    if (!future_result_.valid() || future_result_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
        request = std::make_shared<system_modes_msgs::srv::ChangeMode::Request>();
        request->mode_name = mode_msg.label;
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
        msg->source = "execution_mode_change";
        msg->adaptation_status = AdaptationStatus::STATUS_ADAPTATION_FINISHED;

        if (parseModeChangeResponse(*response))
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

bool ModeChangeServiceHandler::parseModeChangeResponse(const system_modes_msgs::srv::ChangeMode_Response &response)
{
    failed_adaptations.clear();
    if (!response.success)
    {
        failed_adaptations.push_back(current_adaptations[0]);
    }
    return failed_adaptations.size() == 0;
}