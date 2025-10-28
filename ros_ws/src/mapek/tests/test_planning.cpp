#include <gtest/gtest.h>
#include <mapek/planning.hpp>
#include "test_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <system_interfaces/msg/adaptation_type.hpp>

// Utility function to reset environment variables between tests
void resetEnvVars() {
    unsetenv("CONSIDER_DEPENDENCIES");
    unsetenv("CONSIDER_CRITICALITY_LEVEL");
    unsetenv("CONSIDER_COST_FUNCTION");
}

TEST(PlanningEnvironmentTest, TestDefaultBooleanFlags) {
    resetEnvVars();

    Planning planning;
    ASSERT_FALSE(planning.getConsiderDependencies());
    ASSERT_FALSE(planning.getConsiderCriticalityLevel());
    ASSERT_FALSE(planning.getConsiderCostFunction());
}

TEST(PlanningEnvironmentTest, TestConsiderDependenciesTrue) {
    resetEnvVars();
    setenv("CONSIDER_DEPENDENCIES", "true", 1);

    Planning planning;
    ASSERT_TRUE(planning.getConsiderDependencies());
}

TEST(PlanningEnvironmentTest, TestConsiderCriticalityLevelTrue) {
    resetEnvVars();
    setenv("CONSIDER_CRITICALITY_LEVEL", "true", 1);

    Planning planning;
    ASSERT_TRUE(planning.getConsiderCriticalityLevel());
}

TEST(PlanningEnvironmentTest, TestConsiderCostFunctionTrue) {
    resetEnvVars();
    setenv("CONSIDER_COST_FUNCTION", "true", 1);

    Planning planning;
    ASSERT_TRUE(planning.getConsiderCostFunction());
}

TEST(PlanningEnvironmentTest, TestConsiderDependenciesFalse) {
    resetEnvVars();
    setenv("CONSIDER_DEPENDENCIES", "false", 1);

    Planning planning;
    ASSERT_FALSE(planning.getConsiderDependencies());
}

TEST(PlanningEnvironmentTest, TestConsiderCriticalityLevelFalse) {
    resetEnvVars();
    setenv("CONSIDER_CRITICALITY_LEVEL", "false", 1);

    Planning planning;
    ASSERT_FALSE(planning.getConsiderCriticalityLevel());
}

TEST(PlanningEnvironmentTest, TestConsiderCostFunctionFalse) {
    resetEnvVars();
    setenv("CONSIDER_COST_FUNCTION", "false", 1);

    Planning planning;
    ASSERT_FALSE(planning.getConsiderCostFunction());
}


TEST(test_engel_planning, test_engel_graph_simple_rules_no_health_deps)
{
    MAPEK_Graph graph = get_engel_graph();    
    std::vector<RulePtr> rules = get_engel_simple_rules();
    set_health_status(rules, graph);
    setenv("CONSIDER_CRITICALITY_LEVEL", "false", 1);
    setenv("CONSIDER_DEPENDENCIES", "false", 1);
    Planning planning = Planning();
    std::vector<Strategy> strategies = planning.selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_FAILURE);
    ASSERT_EQ(strategies.size(), 1);

    strategies = planning.selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_DEGRADED);
    ASSERT_EQ(strategies.size(), 1);

    strategies = planning.selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_OK);
    ASSERT_EQ(strategies.size(), 1);
}

TEST(test_engel_planning, test_engel_graph_simple_rules_health_deps)
{    
    MAPEK_Graph graph = get_engel_graph();    
    std::vector<RulePtr> rules = get_engel_simple_rules();
    set_health_status(rules, graph);
    setenv("CONSIDER_CRITICALITY_LEVEL", "true", 1);
    setenv("CONSIDER_DEPENDENCIES", "true", 1);
    Planning planning = Planning();
    std::vector<Strategy> strategies = planning.selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_FAILURE);
    ASSERT_EQ(strategies.size(), 1);

    strategies = planning.selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_DEGRADED);
    ASSERT_EQ(strategies.size(), 1);

    strategies = planning.selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_OK);
    ASSERT_EQ(strategies.size(), 1);
}

TEST(test_planning, test_simple_graph_simple_rules_no_health_deps)
{
    MAPEK_Graph graph = get_simple_graph();
    std::vector<RulePtr> rules = get_simple_rules();
    set_health_status(rules, graph);
    setenv("CONSIDER_CRITICALITY_LEVEL", "false", 1);
    setenv("CONSIDER_DEPENDENCIES", "false", 1);
    Planning planning = Planning();

    std::vector<Strategy> strategies = planning.selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_FAILURE);
    ASSERT_EQ(strategies.size(), 2);

    strategies = planning.selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_DEGRADED);
    ASSERT_EQ(strategies.size(), 1);

    strategies = planning.selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_OK);
    ASSERT_EQ(strategies.size(), 0);
}

TEST(test_planning, test_simple_graph_simple_rules_health_deps)
{
    MAPEK_Graph graph = get_simple_graph();
    std::vector<RulePtr> rules = get_simple_rules();
    set_health_status(rules, graph);
    setenv("CONSIDER_CRITICALITY_LEVEL", "true", 1);
    setenv("CONSIDER_DEPENDENCIES", "true", 1);
    Planning planning = Planning();
    std::vector<Strategy> strategies = planning.selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_FAILURE);
    ASSERT_EQ(strategies.size(), 2);

    strategies = planning.selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_DEGRADED);
    ASSERT_EQ(strategies.size(), 0);

    strategies = planning.selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_OK);
    ASSERT_EQ(strategies.size(), 0);
}

TEST(test_planning, test_engel_bad_seg_fusion_dead)
{
    MAPEK_Graph graph = get_engel_graph();
    ENGELRuleConfig config;
    config.bad_segmentation = true;
    config.fusion_dead = true;
    std::vector<RulePtr> rules = get_engel_configurable_rules(config);
    set_health_status(rules, graph);
    setenv("CONSIDER_CRITICALITY_LEVEL", "true", 1);
    setenv("CONSIDER_DEPENDENCIES", "true", 1);
    Planning planning = Planning();
    std::vector<Strategy> strategies = planning.selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_FAILURE);
    ASSERT_EQ(strategies.size(), 1);

    strategies = planning.selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_DEGRADED);
    ASSERT_EQ(strategies.size(), 0);

    resetEnvVars();
    strategies = planning.selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_OK);
    ASSERT_EQ(strategies.size(), 0);

    setenv("CONSIDER_CRITICALITY_LEVEL", "false", 1);
    setenv("CONSIDER_DEPENDENCIES", "false", 1);

    planning.reset();
    ASSERT_FALSE(planning.getConsiderDependencies());
    ASSERT_FALSE(planning.getConsiderCriticalityLevel());

    strategies = planning.selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_FAILURE);
    ASSERT_EQ(strategies.size(), 1);

    strategies = planning.selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_DEGRADED);
    ASSERT_EQ(strategies.size(), 0);

    strategies = planning.selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_OK);
    ASSERT_EQ(strategies.size(), 0);

}

TEST(test_planning, test_engel_bad_seg_depth_dead)
{
    MAPEK_Graph graph = get_engel_graph();
    ENGELRuleConfig config;
    config.bad_segmentation = true;
    config.depth_dead = true;
    std::vector<RulePtr> rules = get_engel_configurable_rules(config);
    set_health_status(rules, graph);
    setenv("CONSIDER_CRITICALITY_LEVEL", "true", 1);
    setenv("CONSIDER_DEPENDENCIES", "true", 1);
    Planning planning = Planning();
    std::vector<Strategy> strategies = planning.selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_FAILURE);
    ASSERT_EQ(strategies.size(), 1);

    strategies = planning.selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_DEGRADED);
    ASSERT_EQ(strategies.size(), 0);

    strategies = planning.selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_OK);
    ASSERT_EQ(strategies.size(), 0);

    resetEnvVars();
    setenv("CONSIDER_CRITICALITY_LEVEL", "false", 1);
    setenv("CONSIDER_DEPENDENCIES", "false", 1);

    planning.reset();
    ASSERT_FALSE(planning.getConsiderDependencies());
    ASSERT_FALSE(planning.getConsiderCriticalityLevel());

    strategies = planning.selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_FAILURE);
    ASSERT_EQ(strategies.size(), 1);

    strategies = planning.selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_DEGRADED);
    ASSERT_EQ(strategies.size(), 1);

    strategies = planning.selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_OK);
    ASSERT_EQ(strategies.size(), 0);

}