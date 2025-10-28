#include "mapek/execution/parametrization.hpp"
#include "mapek/util/logger.hpp"
#include <system_interfaces/msg/experiment_logging.hpp>
#include <system_interfaces/msg/strategy_status.hpp>

using ParametrizationInput = system_interfaces::msg::ParametrizationInput;
using AdaptationType = system_interfaces::msg::AdaptationType;
using AdaptationStatus = system_interfaces::msg::AdaptationStatus;

ParametrizationHandler::ParametrizationHandler(std::shared_ptr<rclcpp::Node> node, const std::string &component_name)
    : node_(node)
{
    std::string stripped_name = string_utils::strip(component_name, '/');
    std::string service_name = "/" + stripped_name + "/set_parameters";
    _component_name = component_name;
    client_ = node_->create_client<rcl_interfaces::srv::SetParameters>(service_name);
    relevantAdaptationTypes_ = {
        AdaptationType::ACTION_DECREASE_PARAMETER,
        AdaptationType::ACTION_INCREASE_PARAMETER,
        AdaptationType::ACTION_SET_PARAMETER};

    experiment_message_ = system_interfaces::msg::ExperimentLogging();
    experiment_message_.source = "execute_parametrization_" + component_name;
    experiment_message_.adaptation_status = system_interfaces::msg::AdaptationStatus::STATUS_NOT_SET;
}

bool ParametrizationHandler::setupRequest(AdaptationMap &adaptations, MAPEK_Graph &graph)
{
    current_adaptations_.clear();
    finished_adaptations.clear();

    if (adaptations.find(_component_name) == adaptations.end())
        return false;
    auto splitAdaptations = adaptations_utils::splitAdaptations(adaptations[_component_name], relevantAdaptationTypes_);
    std::shared_ptr<BTLogger> logger = BTLogger::get_global_logger();
    bool triggeredAdaptations = false;

    for (const auto &adaptation : splitAdaptations.first)
    {
        triggeredAdaptations = true;
        auto param_adaptation = adaptations_utils::toParametrizationAdaptation(adaptation);
        parameters_.push_back(param_adaptation.parameter);
        graph[_component_name].busy(true);

        auto msg = new system_interfaces::msg::ExperimentLogging();
        msg->timestamp = logger->get_time();
        msg->source = "execution_parametrization";
        msg->adaptation_status = AdaptationStatus::STATUS_ADAPTATION_TRIGGERED;
        logger->silent_global(msg);
        adaptations[_component_name] = splitAdaptations.second;
    }
    return triggeredAdaptations;
}

VectorGenericAdaptations ParametrizationHandler::getFailedAdaptations()
{
    return failed_adaptations_;
}

VectorGenericAdaptations ParametrizationHandler::getFinishedAdaptations()
{
    return finished_adaptations;
}

ServiceStatus ParametrizationHandler::tick(MAPEK_Graph &graph)
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
        graph[_component_name].busy(false);
        auto response = future_result_.get();

        std::shared_ptr<BTLogger> logger = BTLogger::get_global_logger();
        auto msg = new system_interfaces::msg::ExperimentLogging();
        msg->timestamp = logger->get_time();
        msg->source = "execution_parametrization";
        msg->adaptation_status = AdaptationStatus::STATUS_ADAPTATION_FINISHED;
        if (parseParameterUpdateResponse(response->results))
        {
            // As long as we do not consider failed adaptations, this will be fine
            finished_adaptations = current_adaptations_;
            msg->success = true;
            logger->silent_global(msg);

            return ServiceStatus::SUCCESS;
        }
        logger->silent_global(msg);

        return ServiceStatus::FAILURE;
    }

    return ServiceStatus::RUNNING;
}

bool ParametrizationHandler::parseParameterUpdateResponse(const std::vector<rcl_interfaces::msg::SetParametersResult> &results)
{
    failed_adaptations_.clear();
    size_t index = 0;
    for (const auto &result : results)
    {
        if (!result.successful)
        {
            failed_adaptations_.push_back(current_adaptations_[index]);
        }
        ++index;
    }
    return failed_adaptations_.empty();
}

void ParametrizationHandler::logExperimentStatus(uint8_t adaptation_type, uint8_t adaptation_status, bool success)
{
    experiment_message_.timestamp = experiment_logger_->get_time();
    experiment_message_.adaptation_type = adaptation_type;
    experiment_message_.adaptation_status = adaptation_status;
    experiment_message_.success = success;
    experiment_logger_->silent_global(&experiment_message_);
}