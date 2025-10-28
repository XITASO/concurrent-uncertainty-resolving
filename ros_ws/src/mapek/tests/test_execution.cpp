#include <mapek/execution/comm_change.hpp>
#include <mapek/execution/lifecycle_change.hpp>
#include <mapek/execution/mode_change.hpp>
#include <mapek/execution/parametrization.hpp>
#include <mapek/execution/redeploy.hpp>
#include <gtest/gtest.h>
#include "test_utils.hpp"
#include <mapek/util/definitions.hpp>

TEST(test_execution, setup_comm_change_graph_valid)
{
    int argc = 1;
    char *argv[] = {strdup("test_execution")}; // Placeholder for program name
    rclcpp::init(argc, argv);
    MAPEK_Graph graph = get_engel_graph();
    GenericAdaptation ga;
    ga.action = system_interfaces::msg::AdaptationType::ACTION_CHANGE_COMMUNICATION;
    ga.parameter = createStringParameterObject("rgb_topic", "rgb_enhanced");
    VectorGenericAdaptations vector = {ga};

    std::string component_name = "fusion";

    rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("execute_comm_change_action_node");
    CommChangeServiceHandler comm_change_handler(node_, component_name);
    AdaptationMap adaptations {};
    adaptations["fusion"] = vector;
    comm_change_handler.setupRequest(adaptations, graph);
    ASSERT_TRUE(graph[component_name].busy());
    rclcpp::shutdown();
}

TEST(test_execution, setup_parametrization_graph_valid)
{
    int argc = 1;
    char *argv[] = {strdup("test_execution")}; // Placeholder for program name
    rclcpp::init(argc, argv);
    MAPEK_Graph graph = get_engel_graph();
    GenericAdaptation ga;
    ga.action = system_interfaces::msg::AdaptationType::ACTION_SET_PARAMETER;
    ga.parameter = createStringParameterObject("rgb_topic", "rgb_enhanced");
    VectorGenericAdaptations vector = {ga};

    std::string component_name = "fusion";
    rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("execute_param_change_action_node");
    ParametrizationHandler param_handler(node_, component_name);
    AdaptationMap adaptations {};
    adaptations["fusion"] = vector;
    param_handler.setupRequest(adaptations, graph);
    ASSERT_TRUE(graph[component_name].busy());
    rclcpp::shutdown();
}

TEST(test_execution, setup_mode_change_graph_valid)
{
    int argc = 1;
    char *argv[] = {strdup("test_execution")}; // Placeholder for program name
    rclcpp::init(argc, argv);
    MAPEK_Graph graph = get_engel_graph();
    GenericAdaptation ga;
    ga.action = system_interfaces::msg::AdaptationType::ACTION_CHANGE_MODE;
    ga.parameter = createStringParameterObject("rgb_topic", "rgb_enhanced");
    VectorGenericAdaptations vector = {ga};

    std::string component_name = "fusion";
    rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("execute_mode_change_action_node");
    ModeChangeServiceHandler handler(node_, component_name);
    AdaptationMap adaptations {};
    adaptations["fusion"] = vector;
    handler.setupRequest(adaptations, graph);
    ASSERT_TRUE(graph[component_name].busy());
    rclcpp::shutdown();
}

TEST(test_execution, setup_lc_graph_valid)
{
    int argc = 1;
    char *argv[] = {strdup("test_execution")}; // Placeholder for program name
    rclcpp::init(argc, argv);
    MAPEK_Graph graph = get_engel_graph();
    GenericAdaptation ga;
    ga.action = system_interfaces::msg::AdaptationType::ACTION_ACTIVATE;
    VectorGenericAdaptations vector = {ga};

    std::string component_name = "fusion";
    rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("execute_lc_change_action_node");
    LifecycleChangeServiceHandler handler(node_, component_name);
    AdaptationMap adaptations {};
    adaptations["fusion"] = vector;
    handler.setupRequest(adaptations, graph);
    ASSERT_TRUE(graph[component_name].busy());
    rclcpp::shutdown();
}

TEST(test_execution, setup_redeploy_graph_valid)
{
    int argc = 1;
    char *argv[] = {strdup("test_execution")}; // Placeholder for program name
    rclcpp::init(argc, argv);
    MAPEK_Graph graph = get_engel_graph();
    GenericAdaptation ga;
    ga.action = system_interfaces::msg::AdaptationType::ACTION_REDEPLOY;
    VectorGenericAdaptations vector = {ga};

    std::string component_name = "fusion";
    rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("execute_lc_change_action_node");
    RedeployServiceHandler handler(node_, component_name);
    AdaptationMap adaptations {};
    adaptations["fusion"] = vector;
    handler.setupRequest(adaptations, graph);
    ASSERT_TRUE(graph[component_name].busy());
    rclcpp::shutdown();
}

TEST(test_execution, setup_comm_change_graph_no_relevant_adaptation)
{
    int argc = 1;
    char *argv[] = {strdup("test_execution")}; // Placeholder for program name
    rclcpp::init(argc, argv);
    MAPEK_Graph graph = get_engel_graph();
    GenericAdaptation ga;
    ga.action = system_interfaces::msg::AdaptationType::ACTION_SET_PARAMETER;
    ga.parameter = createStringParameterObject("rgb_topic", "rgb_enhanced");
    VectorGenericAdaptations vector = {ga};

    std::string component_name = "fusion";

    rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("execute_comm_change_action_node");
    CommChangeServiceHandler comm_change_handler(node_, component_name);
    AdaptationMap adaptations {};
    adaptations["fusion"] = vector;
    comm_change_handler.setupRequest(adaptations, graph);
    ASSERT_FALSE(graph[component_name].busy());
    rclcpp::shutdown();
}

TEST(test_execution, setup_parametrization_graph_no_relevant_adaptation)
{
    int argc = 1;
    char *argv[] = {strdup("test_execution")}; // Placeholder for program name
    rclcpp::init(argc, argv);
    MAPEK_Graph graph = get_engel_graph();
    GenericAdaptation ga;
    ga.action = system_interfaces::msg::AdaptationType::ACTION_ACTIVATE;
    VectorGenericAdaptations vector = {ga};

    std::string component_name = "fusion";
    rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("execute_param_change_action_node");
    ParametrizationHandler param_handler(node_, component_name);
    AdaptationMap adaptations {};
    adaptations["fusion"] = vector;
    param_handler.setupRequest(adaptations, graph);
    ASSERT_FALSE(graph[component_name].busy());
    rclcpp::shutdown();
}

TEST(test_execution, setup_mode_change_graph_no_relevant_adaptation)
{
    int argc = 1;
    char *argv[] = {strdup("test_execution")}; // Placeholder for program name
    rclcpp::init(argc, argv);
    MAPEK_Graph graph = get_engel_graph();
    GenericAdaptation ga;
    ga.action = system_interfaces::msg::AdaptationType::ACTION_ACTIVATE;
    VectorGenericAdaptations vector = {ga};

    std::string component_name = "fusion";
    rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("execute_mode_change_action_node");
    ModeChangeServiceHandler handler(node_, component_name);
    AdaptationMap adaptations {};
    adaptations["fusion"] = vector;
    handler.setupRequest(adaptations, graph);
    ASSERT_FALSE(graph[component_name].busy());
    rclcpp::shutdown();
}

TEST(test_execution, setup_lc_graph_valid_no_relevant_adaptaion)
{
    int argc = 1;
    char *argv[] = {strdup("test_execution")}; // Placeholder for program name
    rclcpp::init(argc, argv);
    MAPEK_Graph graph = get_engel_graph();
    GenericAdaptation ga;
    ga.action = system_interfaces::msg::AdaptationType::ACTION_REDEPLOY;
    VectorGenericAdaptations vector = {ga};

    std::string component_name = "fusion";
    rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("execute_lc_change_action_node");
    LifecycleChangeServiceHandler handler(node_, component_name);
    AdaptationMap adaptations {};
    adaptations["fusion"] = vector;
    handler.setupRequest(adaptations, graph);
    ASSERT_FALSE(graph[component_name].busy());
    rclcpp::shutdown();
}

TEST(test_execution, setup_redeploy_graph_valid_no_relevant_adaptation)
{
    int argc = 1;
    char *argv[] = {strdup("test_execution")}; // Placeholder for program name
    rclcpp::init(argc, argv);
    MAPEK_Graph graph = get_engel_graph();
    GenericAdaptation ga;
    ga.action = system_interfaces::msg::AdaptationType::ACTION_ACTIVATE;
    VectorGenericAdaptations vector = {ga};

    std::string component_name = "fusion";
    rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("execute_lc_change_action_node");
    RedeployServiceHandler handler(node_, component_name);
    AdaptationMap adaptations {};
    adaptations["fusion"] = vector;
    handler.setupRequest(adaptations, graph);
    ASSERT_FALSE(graph[component_name].busy());
    rclcpp::shutdown();
}