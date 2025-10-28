#ifndef COMM_CHANGE_SERVICE_HANDLER_HPP
#define COMM_CHANGE_SERVICE_HANDLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "system_interfaces/msg/adaptation_status.hpp"
#include "system_interfaces/msg/adaptation_type.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "mapek/util/adaptation_utils.hpp"
#include "mapek/util/string_utils.hpp"

#include <mapek/util/definitions.hpp>
#include <mapek/util/graph.hpp>

class CommChangeServiceHandler
{
public:
    CommChangeServiceHandler(rclcpp::Node::SharedPtr node, const std::string &component_name);

    bool setupRequest(AdaptationMap&adaptations, MAPEK_Graph &graph);
    VectorGenericAdaptations getFailedAdaptations();
    VectorGenericAdaptations getFinishedAdaptations();
    ServiceStatus tick(MAPEK_Graph &graph);

private:
    rclcpp::Node::SharedPtr node_;
    std::vector<uint8_t> relevantAdaptationTypes;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client_;
    std::vector<rcl_interfaces::msg::Parameter> parameters_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future_result_;
    VectorGenericAdaptations current_adaptations;
    VectorGenericAdaptations failed_adaptations;
    VectorGenericAdaptations finished_adaptations {};
    std::string _component_name;

    bool parseParameterUpdateResponse(const std::vector<rcl_interfaces::msg::SetParametersResult> &results);
};

#endif // COMM_CHANGE_SERVICE_HANDLER_HPP