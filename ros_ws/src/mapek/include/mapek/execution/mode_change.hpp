#ifndef MODE_CHANGE_SERVICE_HANDLER_HPP
#define MODE_CHANGE_SERVICE_HANDLER_HPP

#include <rclcpp/rclcpp.hpp>

#include <system_modes_msgs/srv/change_mode.hpp>

#include <system_interfaces/msg/adaptation_status.hpp>
#include <system_interfaces/msg/adaptation_type.hpp>

#include <mapek/util/definitions.hpp>
#include <mapek/util/adaptation_utils.hpp>
#include <mapek/util/string_utils.hpp>
#include <mapek/util/graph.hpp>

class ModeChangeServiceHandler
{
public:
    ModeChangeServiceHandler(rclcpp::Node::SharedPtr node, const std::string &component_name);

    bool setupRequest(AdaptationMap&adaptations, MAPEK_Graph &graph);
    VectorGenericAdaptations getFailedAdaptations();
    VectorGenericAdaptations getFinishedAdaptations();
    ServiceStatus tick(MAPEK_Graph &graph);

private:
    rclcpp::Node::SharedPtr node_;
    std::vector<uint8_t> relevantAdaptationTypes;
    rclcpp::Client<system_modes_msgs::srv::ChangeMode>::SharedPtr client_;
     
    rclcpp::Client<system_modes_msgs::srv::ChangeMode>::SharedFuture future_result_;
    std::shared_ptr<system_modes_msgs::srv::ChangeMode_Request> request;
    system_modes_msgs::msg::Mode mode_msg;
    VectorGenericAdaptations current_adaptations;
    VectorGenericAdaptations failed_adaptations;
    VectorGenericAdaptations finished_adaptations {};
    std::vector<lifecycle_msgs::msg::Transition> changes {};
    std::string _component_name;

    bool parseModeChangeResponse(const system_modes_msgs::srv::ChangeMode_Response &response);
};

#endif //LIFECYCLE_CHANGE_SERVICE_HANDLER_HPP