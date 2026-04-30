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

// Tests for validateStrategy method
TEST(test_planning_validateStrategy, test_activate_on_active_node)
{
    // Test that activating an already active node returns false
    MAPEK_Graph graph = get_simple_graph();
    
    // Set node A to ACTIVE state (state 3)
    auto& nodeA = const_cast<GraphNode&>(graph.nodes()->at("A"));
    nodeA.lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    
    // Create a strategy that activates node A
    Adaptation adaptation(
        "A",
        system_interfaces::msg::AdaptationType::ACTION_ACTIVATE,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_ACTIVATE;
            return ga;
        });
    Strategy strategy({adaptation}, "activate_A", 1, 0.95);
    
    Planning planning;
    ASSERT_FALSE(planning.validateStrategy(graph, strategy));
}

TEST(test_planning_validateStrategy, test_activate_on_inactive_node)
{
    // Test that activating an inactive node returns true
    MAPEK_Graph graph = get_simple_graph();
    
    // Set node A to INACTIVE state (state 2)
    auto& nodeA = const_cast<GraphNode&>(graph.nodes()->at("A"));
    nodeA.lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    
    // Create a strategy that activates node A
    Adaptation adaptation(
        "A",
        system_interfaces::msg::AdaptationType::ACTION_ACTIVATE,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_ACTIVATE;
            return ga;
        });
    Strategy strategy({adaptation}, "activate_A", 1, 0.95);
    
    Planning planning;
    ASSERT_TRUE(planning.validateStrategy(graph, strategy));
}

TEST(test_planning_validateStrategy, test_activate_on_unconfigured_node)
{
    // Test that activating an unconfigured node returns true
    MAPEK_Graph graph = get_simple_graph();
    
    // Set node A to UNCONFIGURED state (state 1)
    auto& nodeA = const_cast<GraphNode&>(graph.nodes()->at("A"));
    nodeA.lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    
    // Create a strategy that activates node A
    Adaptation adaptation(
        "A",
        system_interfaces::msg::AdaptationType::ACTION_ACTIVATE,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_ACTIVATE;
            return ga;
        });
    Strategy strategy({adaptation}, "activate_A", 1, 0.95);
    
    Planning planning;
    ASSERT_TRUE(planning.validateStrategy(graph, strategy));
}

TEST(test_planning_validateStrategy, test_deactivate_on_active_node)
{
    // Test that deactivating an active node returns true
    MAPEK_Graph graph = get_simple_graph();
    
    // Set node B to ACTIVE state (state 3)
    auto& nodeB = const_cast<GraphNode&>(graph.nodes()->at("B"));
    nodeB.lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    
    // Create a strategy that deactivates node B
    Adaptation adaptation(
        "B",
        system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE;
            return ga;
        });
    Strategy strategy({adaptation}, "deactivate_B", 2, 0.90);
    
    Planning planning;
    ASSERT_TRUE(planning.validateStrategy(graph, strategy));
}

TEST(test_planning_validateStrategy, test_deactivate_on_inactive_node)
{
    // Test that deactivating an already inactive node returns false
    MAPEK_Graph graph = get_simple_graph();
    
    // Set node B to INACTIVE state (state 2)
    auto& nodeB = const_cast<GraphNode&>(graph.nodes()->at("B"));
    nodeB.lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    
    // Create a strategy that deactivates node B
    Adaptation adaptation(
        "B",
        system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE;
            return ga;
        });
    Strategy strategy({adaptation}, "deactivate_B", 2, 0.90);
    
    Planning planning;
    ASSERT_FALSE(planning.validateStrategy(graph, strategy));
}

TEST(test_planning_validateStrategy, test_deactivate_on_unconfigured_node)
{
    // Test that deactivating an unconfigured node returns false
    MAPEK_Graph graph = get_simple_graph();
    
    // Set node B to UNCONFIGURED state (state 1)
    auto& nodeB = const_cast<GraphNode&>(graph.nodes()->at("B"));
    nodeB.lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    
    // Create a strategy that deactivates node B
    Adaptation adaptation(
        "B",
        system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE;
            return ga;
        });
    Strategy strategy({adaptation}, "deactivate_B", 2, 0.90);
    
    Planning planning;
    ASSERT_FALSE(planning.validateStrategy(graph, strategy));
}

TEST(test_planning_validateStrategy, test_multiple_adaptations_all_valid)
{
    // Test strategy with multiple adaptations where all are valid
    MAPEK_Graph graph = get_simple_graph();
    
    // Set node A to INACTIVE and node B to ACTIVE
    auto& nodeA = const_cast<GraphNode&>(graph.nodes()->at("A"));
    nodeA.lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    
    auto& nodeB = const_cast<GraphNode&>(graph.nodes()->at("B"));
    nodeB.lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    
    // Create adaptations
    Adaptation adaptation1(
        "A",
        system_interfaces::msg::AdaptationType::ACTION_ACTIVATE,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_ACTIVATE;
            return ga;
        });
    Adaptation adaptation2(
        "B",
        system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE;
            return ga;
        });
    Strategy strategy({adaptation1, adaptation2}, "multi_strategy", 3, 0.85);
    
    Planning planning;
    ASSERT_TRUE(planning.validateStrategy(graph, strategy));
}

TEST(test_planning_validateStrategy, test_multiple_adaptations_one_invalid)
{
    // Test strategy with multiple adaptations where one is invalid
    MAPEK_Graph graph = get_simple_graph();
    
    // Set node A to ACTIVE (so activation would be invalid) and node B to ACTIVE
    auto& nodeA = const_cast<GraphNode&>(graph.nodes()->at("A"));
    nodeA.lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    
    auto& nodeB = const_cast<GraphNode&>(graph.nodes()->at("B"));
    nodeB.lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    
    // Create adaptations: try to activate an already active node A
    Adaptation adaptation1(
        "A",
        system_interfaces::msg::AdaptationType::ACTION_ACTIVATE,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_ACTIVATE;
            return ga;
        });
    Adaptation adaptation2(
        "B",
        system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE;
            return ga;
        });
    Strategy strategy({adaptation1, adaptation2}, "multi_strategy", 3, 0.85);
    
    Planning planning;
    ASSERT_FALSE(planning.validateStrategy(graph, strategy));
}

TEST(test_planning_validateStrategy, test_non_lifecycle_adaptations)
{
    // Test that non-lifecycle adaptations (e.g., CHANGE_MODE) don't affect validation
    MAPEK_Graph graph = get_simple_graph();
    
    // Set node C to ACTIVE
    auto& nodeC = const_cast<GraphNode&>(graph.nodes()->at("C"));
    nodeC.lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    
    // Create a strategy that changes mode on node C
    Adaptation adaptation(
        "C",
        system_interfaces::msg::AdaptationType::ACTION_CHANGE_MODE,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_CHANGE_MODE;
            return ga;
        });
    Strategy strategy({adaptation}, "change_mode_C", 4, 0.80);
    
    Planning planning;
    // Should return true because non-lifecycle adaptations are always valid
    ASSERT_TRUE(planning.validateStrategy(graph, strategy));
}

TEST(test_planning_validateStrategy, test_component_not_in_graph)
{
    // Test that a strategy with a component not in the graph is handled gracefully
    MAPEK_Graph graph = get_simple_graph();
    
    // Create a strategy that affects a non-existent component
    Adaptation adaptation(
        "NonExistent",
        system_interfaces::msg::AdaptationType::ACTION_ACTIVATE,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_ACTIVATE;
            return ga;
        });
    Strategy strategy({adaptation}, "activate_nonexistent", 5, 0.75);
    
    Planning planning;
    // Should return true because missing components are skipped
    ASSERT_TRUE(planning.validateStrategy(graph, strategy));
}

TEST(test_planning_validateStrategy, test_restart_on_active_node)
{
    // Test that restarting an active node returns true
    MAPEK_Graph graph = get_simple_graph();
    
    // Set node A to ACTIVE state (state 3)
    auto& nodeA = const_cast<GraphNode&>(graph.nodes()->at("A"));
    nodeA.lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    
    // Create a strategy that restarts node A
    Adaptation adaptation(
        "A",
        system_interfaces::msg::AdaptationType::ACTION_RESTART,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_RESTART;
            return ga;
        });
    Strategy strategy({adaptation}, "restart_A", 6, 0.85);
    
    Planning planning;
    ASSERT_TRUE(planning.validateStrategy(graph, strategy));
}

TEST(test_planning_validateStrategy, test_restart_on_inactive_node)
{
    // Test that restarting an inactive node returns false (not worth trying)
    MAPEK_Graph graph = get_simple_graph();
    
    // Set node A to INACTIVE state (state 2)
    auto& nodeA = const_cast<GraphNode&>(graph.nodes()->at("A"));
    nodeA.lifecycle_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    
    // Create a strategy that restarts node A
    Adaptation adaptation(
        "A",
        system_interfaces::msg::AdaptationType::ACTION_RESTART,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_RESTART;
            return ga;
        });
    Strategy strategy({adaptation}, "restart_A", 6, 0.85);
    
    Planning planning;
    ASSERT_FALSE(planning.validateStrategy(graph, strategy));
}