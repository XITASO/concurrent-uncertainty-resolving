#ifndef LIFECYCLE_CHANGE_SERVICE_HANDLER_HPP
#define LIFECYCLE_CHANGE_SERVICE_HANDLER_HPP

#include <rclcpp/rclcpp.hpp>

#include <system_interfaces/srv/set_lifecycle_changes.hpp>
#include <system_interfaces/msg/adaptation_status.hpp>
#include <system_interfaces/msg/adaptation_type.hpp>

#include <mapek/util/definitions.hpp>
#include <mapek/util/adaptation_utils.hpp>
#include <mapek/util/string_utils.hpp>
#include <mapek/util/graph.hpp>

class LifecycleChangeServiceHandler
{
public:
    LifecycleChangeServiceHandler(rclcpp::Node::SharedPtr node, const std::string &component_name);

    bool setupRequest(AdaptationMap&adaptations, MAPEK_Graph &graph);
    VectorGenericAdaptations getFailedAdaptations();
    VectorGenericAdaptations getFinishedAdaptations();
    ServiceStatus tick(MAPEK_Graph &graph);

private:
    rclcpp::Node::SharedPtr node_;
    std::vector<uint8_t> relevantAdaptationTypes;
    rclcpp::Client<system_interfaces::srv::SetLifecycleChanges>::SharedPtr client_;
     
    rclcpp::Client<system_interfaces::srv::SetLifecycleChanges>::SharedFuture future_result_;
    VectorGenericAdaptations current_adaptations;
    VectorGenericAdaptations failed_adaptations;
    VectorGenericAdaptations finished_adaptations {};
    std::vector<lifecycle_msgs::msg::Transition> transitions {};
    std::string _component_name;

    bool parseLifecycleResponse(const system_interfaces::srv::SetLifecycleChanges_Response &response);
};

#endif //LIFECYCLE_CHANGE_SERVICE_HANDLER_HPP