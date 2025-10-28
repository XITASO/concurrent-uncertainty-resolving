#include "mapek/execution/comm_change.hpp"
#include "mapek/util/logger.hpp"
#include <system_interfaces/msg/experiment_logging.hpp>
#include <system_interfaces/msg/strategy_status.hpp>

using AdaptationType = system_interfaces::msg::AdaptationType;
using AdaptationStatus = system_interfaces::msg::AdaptationStatus;
CommChangeServiceHandler::CommChangeServiceHandler(rclcpp::Node::SharedPtr node, const std::string &component_name)
    : node_(node)
{
    // Initialize the service client
    std::string service_name = "/" + string_utils::strip(component_name, '/') + "/set_parameters";
    _component_name = component_name;
    client_ = node_->create_client<rcl_interfaces::srv::SetParameters>(service_name);
    relevantAdaptationTypes = {AdaptationType::ACTION_CHANGE_COMMUNICATION};
}

bool CommChangeServiceHandler::setupRequest(AdaptationMap &adaptations, MAPEK_Graph &graph)
{
    parameters_.clear();
    finished_adaptations.clear();

    if (adaptations.find(_component_name) == adaptations.end())
        return false;
    auto splitAdaptations = adaptations_utils::splitAdaptations(adaptations[_component_name], relevantAdaptationTypes);
    std::shared_ptr<BTLogger> logger = BTLogger::get_global_logger();
    bool triggeredAdaptations = false;

    for (const auto &adaptation : splitAdaptations.first)
    {
        triggeredAdaptations = true;
        current_adaptations.push_back(adaptation);
        auto comm_change = adaptations_utils::toCommChangeAdaptation(adaptation);
        rcl_interfaces::msg::Parameter param_msg;
        param_msg.name = comm_change.communication_name.data;
        param_msg.value.string_value = comm_change.value.data;
        param_msg.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        parameters_.push_back(param_msg);

        auto msg = new system_interfaces::msg::ExperimentLogging();
        msg->timestamp = logger->get_time();
        msg->source = "execution_comm_change";
        msg->adaptation_status = AdaptationStatus::STATUS_ADAPTATION_TRIGGERED;
        msg->adaptation_type = AdaptationType::ACTION_CHANGE_COMMUNICATION;
        logger->silent_global(msg);

        // if we find something, then we set ourselves to busy
        graph[_component_name].busy(true);
        adaptations[_component_name] = splitAdaptations.second;
    }
    return triggeredAdaptations;
}

VectorGenericAdaptations CommChangeServiceHandler::getFailedAdaptations()
{
    return failed_adaptations;
}

VectorGenericAdaptations CommChangeServiceHandler::getFinishedAdaptations()
{
    return finished_adaptations;
}

ServiceStatus CommChangeServiceHandler::tick(MAPEK_Graph &graph)
{
    if (!future_result_.valid() || future_result_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        request->parameters = parameters_;

        future_result_ = client_->async_send_request(request);
        return ServiceStatus::RUNNING;
    }
    rclcpp::spin_some(node_);

    if (future_result_.valid() && future_result_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
        auto response = future_result_.get();
        graph[_component_name].busy(false);

        std::shared_ptr<BTLogger> logger = BTLogger::get_global_logger();
        auto msg = new system_interfaces::msg::ExperimentLogging();
        msg->timestamp = logger->get_time();
        msg->source = "execution_comm_change";
        msg->adaptation_status = AdaptationStatus::STATUS_ADAPTATION_FINISHED;

        if (parseParameterUpdateResponse(response->results))
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

bool CommChangeServiceHandler::parseParameterUpdateResponse(const std::vector<rcl_interfaces::msg::SetParametersResult> &results)
{
    failed_adaptations.clear();
    int idx = 0;
    for (const auto &result : results)
    {
        if (!result.successful)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to change comm type: %s", result.reason.c_str());
            failed_adaptations.push_back(current_adaptations[idx]);
        }
        idx++;
    }
    return failed_adaptations.size() == 0;
}