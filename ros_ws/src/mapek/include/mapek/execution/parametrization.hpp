#include "system_interfaces/msg/parametrization_input.hpp"
#include "mapek/util/string_utils.hpp"
#include "mapek/util/adaptation_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <system_interfaces/msg/generic_adaptation.hpp>
#include <system_interfaces/msg/adaptation_type.hpp>
#include <system_interfaces/msg/adaptation_status.hpp>
#include <system_interfaces/msg/experiment_logging.hpp>
#include "mapek/util/logger.hpp"

#include <mapek/util/definitions.hpp>
#include <mapek/util/graph.hpp>


class ParametrizationHandler
{
public:
    ParametrizationHandler(std::shared_ptr<rclcpp::Node> node, const std::string &component_name);

    bool setupRequest(AdaptationMap&adaptations, MAPEK_Graph &graph);
    VectorGenericAdaptations getFailedAdaptations();
    VectorGenericAdaptations getFinishedAdaptations();
    ServiceStatus tick(MAPEK_Graph &graph);

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::vector<uint8_t> relevantAdaptationTypes_;
    std::shared_ptr<BTLogger> experiment_logger_;
    system_interfaces::msg::ExperimentLogging experiment_message_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future_result_;
    std::vector<rcl_interfaces::msg::Parameter> parameters_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client_;

    VectorGenericAdaptations current_adaptations_;
    VectorGenericAdaptations failed_adaptations_;
    VectorGenericAdaptations finished_adaptations {};
    std::string _component_name;
    
    bool parseParameterUpdateResponse(const std::vector<rcl_interfaces::msg::SetParametersResult> &results);
    void logExperimentStatus(uint8_t adaptation_type, uint8_t adaptation_status, bool success);
};
