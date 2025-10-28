#include <mapek/execution/redeploy.hpp>
#include "mapek/util/logger.hpp"
#include <system_interfaces/msg/experiment_logging.hpp>
#include <system_interfaces/msg/strategy_status.hpp>

using AdaptationType = system_interfaces::msg::AdaptationType;
using AdaptationStatus = system_interfaces::msg::AdaptationStatus;
RedeployServiceHandler::RedeployServiceHandler(rclcpp::Node::SharedPtr node, const std::string &component_name)
    : node_(node)
{
    // Initialize the service client
    std::string service_name = "/" + string_utils::strip(component_name, '/') + "/change_state";
    client_ = node_->create_client<lifecycle_msgs::srv::ChangeState>(service_name);
    relevantAdaptationTypes = {AdaptationType::ACTION_REDEPLOY};
    _component_name = component_name;

    // Extract the executable name
    size_t last_slash_pos = component_name.find_last_of("/");
    executable_name = component_name.substr(last_slash_pos + 1);

    // Extract the namespace name
    size_t first_slash_pos = component_name.find_first_of("/");
    size_t second_slash_pos = component_name.find_first_of("/", first_slash_pos + 1);
    package_name = component_name.substr(first_slash_pos + 1, second_slash_pos - first_slash_pos - 1);
}

bool RedeployServiceHandler::setupRequest(AdaptationMap &adaptations, MAPEK_Graph &graph)
{
    finished_adaptations.clear();
    if (adaptations.find(_component_name) == adaptations.end())
        return false;
    auto splitAdaptations = adaptations_utils::splitAdaptations(adaptations[_component_name], relevantAdaptationTypes);
    std::shared_ptr<BTLogger> logger = BTLogger::get_global_logger();
    bool triggeredAdaptations = false;
    if (splitAdaptations.first.size() > 0)
    {
        triggeredAdaptations = true;
        adaptation = splitAdaptations.first[0];
        graph[_component_name].busy(true);

        auto msg = new system_interfaces::msg::ExperimentLogging();
        msg->timestamp = logger->get_time();
        msg->source = "execution_redeploy";
        msg->adaptation_type = AdaptationType::ACTION_REDEPLOY;
        msg->adaptation_status = AdaptationStatus::STATUS_ADAPTATION_TRIGGERED;
        logger->silent_global(msg);
        adaptations[_component_name] = splitAdaptations.second;
    }

    return triggeredAdaptations;
}

VectorGenericAdaptations RedeployServiceHandler::getFailedAdaptations()
{
    return failed_adaptations;
}

VectorGenericAdaptations RedeployServiceHandler::getFinishedAdaptations()
{
    return finished_adaptations;
}

ServiceStatus RedeployServiceHandler::tick(MAPEK_Graph &graph)
{
    if (!future_result_.valid() || future_result_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN;
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
        msg->source = "execution_redeploy";
        msg->adaptation_status = AdaptationStatus::STATUS_ADAPTATION_FINISHED;

        if (parseLifecycleResponse(*response))
        {
            // As long as we do not consider failed adaptations, this will be fine
            finished_adaptations.push_back(adaptation);
            restart_ros2_node(package_name, executable_name);
            msg->success = true;
            logger->silent_global(msg);
            return ServiceStatus::SUCCESS;
        }
        logger->silent_global(msg);
        return ServiceStatus::FAILURE;
    }

    return ServiceStatus::RUNNING;
}

bool RedeployServiceHandler::parseLifecycleResponse(const lifecycle_msgs::srv::ChangeState_Response &response)
{
    failed_adaptations.clear();
    if (!response.success)
    {
        failed_adaptations.push_back(adaptation);
    }
    return failed_adaptations.size() == 0;
}

/**
 * Restarts a ROS2 node using the specified package and executable.
 * There has to be a launch file for this specific ROS node present that has the correct name.
 * E.g. if the ROS node name is /<namespace>/<node_name>, then there has to be a launch file called <node name>.launch.py
 * in the <namespace> ROS package
 *
 * @param package The name of the package containing the launch file.
 * @param executable The name of the executable to be launched.
 *
 * @return A boolean value indicating whether the restart operation was successful.
 */
bool RedeployServiceHandler::restart_ros2_node(const std::string &package, const std::string &executable)
{
    // restart with launch file instead of run command to ensure parameters stay the same
    std::string command = "ros2 launch " + package + " " + executable + ".launch.py &";
    int ret_code = system(command.c_str());
    return (ret_code == 0);
}