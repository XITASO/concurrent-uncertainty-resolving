#include <iostream>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "system_interfaces/msg/heartbeat.hpp"
#include "system_interfaces/msg/generic_adaptation.hpp"
#include <system_interfaces/msg/adaptation_type.hpp>
#include <mapek/planning.hpp>
#include <mapek/util/definitions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mapek/rules/expression/ExpressionFactory.hpp>
#include <mapek/rules/RuleParser.hpp>
#include <mapek/rules/Rule.hpp>
#include <mapek/analyzer.hpp>
#include <mapek/util/parameter_message_utils.hpp>


// Include the classes and function from the previous implementation
#include <mapek/planning.hpp>
#include <mapek/util/system_analyzer.hpp>
#include <mapek/monitoring.hpp>

void set_health_status(const std::vector<Rule> rules, MAPEK_Graph& graph)
{
    for (const auto& rule : rules)
    {
        for (const auto& strategy : rule.getStrategies())
        {
            for (const std::string& component : strategy.getAffectedComponents())
            {
                graph[component].health_status(rule.getCriticalityLevel());
            }
        }

    }
}

MAPEK_Graph get_simple_graph()
{
    // Create a graph representing system nodes
    MAPEK_Graph systemGraph;

    // Add nodes to the graph
    GraphNode nodeA("A");
    GraphNode nodeB("B");
    GraphNode nodeC("C");

    systemGraph.addNode(nodeA);
    systemGraph.addNode(nodeB);
    systemGraph.addNode(nodeC);

    // Add edges representing dependencies
    systemGraph.addEdge("A", "B");
    systemGraph.addEdge("B", "C");
    return systemGraph;
}

std::vector<Rule> get_simple_rules() {

    Adaptation adaptation1(
        "A", 
        system_interfaces::msg::AdaptationType::ACTION_ACTIVATE,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_ACTIVATE;
            return ga;
        }
    );
    Adaptation adaptation2(
        "B", 
        system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE;
            return ga;
        }
    );
    Adaptation adaptation3(
        "C", 
        system_interfaces::msg::AdaptationType::ACTION_CHANGE_MODE,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_CHANGE_MODE;
            return ga;
        }
    );
    Adaptation adaptation4(
        "D", 
        system_interfaces::msg::AdaptationType::ACTION_CHANGE_COMMUNICATION,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_CHANGE_COMMUNICATION;
            return ga;
        }
    );

    // Create strategies
    Strategy strategy1({ adaptation1 }, "s1", 1, 0.95);
    Strategy strategy2({ adaptation2 }, "s2", 2, 0.90);
    Strategy strategy3({ adaptation3 }, "s3", 3, 0.85);
    Strategy strategy4({ adaptation4 }, "s4", 4, 0.94);

    auto trigger = std::make_shared<Expression>(Token("true", Token::Type::BOOL));

    // Create MAPEK rules
    //Rule rule1("r1", trigger, { strategy1, strategy4 }, Heartbeat::HB_STATUS_FAILURE, 1, {1,1});
    //Rule rule2("r2", trigger, { strategy2, }, Heartbeat::HB_STATUS_DEGRADED, 2, {1,1});
    //Rule rule3("r3", trigger, { strategy3 }, Heartbeat::HB_STATUS_FAILURE, 3, {1,1});

    //std::vector<Rule> rules = { rule1, rule2, rule3 };
    std::vector<Rule> rules = { };
    return rules;
}

ValueStorePtr getValueStore()
{
    auto value_store = std::make_shared<ValueStore>();
    (*value_store)["double_val"] = param_utils::getParamValue(1.0);
    (*value_store)["string_val"] = param_utils::getParamValue("hello");
    (*value_store)["bool_val"] = param_utils::getParamValue(true);
    return value_store;
}

MAPEK_Graph get_engel_graph()
{
    // Create a graph representing system nodes
    MAPEK_Graph systemGraph;

    // Add nodes to the graph
    GraphNode nodeA("camera");
    GraphNode nodeB("depth");
    GraphNode nodeC("enhancement");
    GraphNode nodeD("fusion");
    GraphNode nodeE("segmentation");

    systemGraph.addNode(nodeA);
    systemGraph.addNode(nodeB);
    systemGraph.addNode(nodeC);
    systemGraph.addNode(nodeD);
    systemGraph.addNode(nodeE);

    // Add edges representing dependencies
    systemGraph.addEdge("segmentation", "fusion");
    systemGraph.addEdge("fusion", "enhancement");
    systemGraph.addEdge("fusion", "depth");
    systemGraph.addEdge("enhancement", "camera");
    return systemGraph;
}

std::vector<Rule> get_engel_simple_rules()
{
 Adaptation adaptation1(
        "camera", 
        system_interfaces::msg::AdaptationType::ACTION_ACTIVATE,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_ACTIVATE;
            return ga;
        }
    );
    Adaptation adaptation2(
        "enhancement", 
        system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE;
            return ga;
        }
    );
    Adaptation adaptation3(
        "fusion",
        system_interfaces::msg::AdaptationType::ACTION_CHANGE_MODE,
        [](ValueStorePtr) {
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_CHANGE_MODE;
            return ga;
        }
    );
    Adaptation adaptation4(
        "depth",
        system_interfaces::msg::AdaptationType::ACTION_CHANGE_COMMUNICATION,
        [](ValueStorePtr) {
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_CHANGE_COMMUNICATION;
            return ga;
        }
    );

    // Create strategies
    Strategy strategy1({ adaptation1 }, "s1", 1, 0.95);
    Strategy strategy2({ adaptation2 }, "s2", 2, 0.90);
    Strategy strategy3({ adaptation3 }, "s3", 3, 0.85);
    Strategy strategy4({ adaptation4 }, "s4", 4, 0.94);

    auto trigger = std::make_shared<Expression>(Token("true", Token::Type::BOOL));

    //Rule rule1("bad_camera_data", trigger, { strategy1, strategy4 }, Heartbeat::HB_STATUS_FAILURE, 1, {1,1});
    //Rule rule2("no_fusion_results", trigger, { strategy2, }, Heartbeat::HB_STATUS_DEGRADED, 2, {1,1});
    //Rule rule3("add_depth", trigger, { strategy3 }, Heartbeat::HB_STATUS_OK, 3, {1,1});

    //std::vector<Rule> rules = { rule1, rule2, rule3 };
    std::vector<Rule> rules = { };
    return rules;
}



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto value_store = getValueStore();
    std::vector<RulePtr> darules;
    MAPEK_Graph graph;
    graph.addNode(GraphNode("component"));
    Adaptation adaptation(
        "component", 
        system_interfaces::msg::AdaptationType::ACTION_SET_PARAMETER, 
        [](ValueStorePtr){return GenericAdaptation{};});

    // Create strategies
    Strategy strategy1({adaptation}, "strat1", 1, 0.95);
    Strategy strategy2({adaptation}, "strat2", 2, 0.95);
    // ASSERT_FALSE(strategy1.didWeTryThis());
    // ASSERT_FALSE(strategy2.didWeTryThis());
    std::vector<Strategy> strategies = {strategy1, strategy2};
    auto trigger = std::make_shared<Expression>(Token("bool_val", Token::Type::VAR));
    
    darules.push_back(std::make_shared<Rule>("rule", trigger, std::vector<Strategy>{strategies},Heartbeat::HB_STATUS_FAILURE, 1, Rule::FilterPolicy{1,1}, Rule::TriggerPolicy::ON_TICK));
    // // ASSERT_FALSE(darules[0]->getStrategies()[0].didWeTryThis());
    // // ASSERT_FALSE(darules[0]->getStrategies()[1].didWeTryThis());

    // ASSERT_EQ(darules[0]->getStrategies()[0].getName(), "strat1");
    // ASSERT_EQ(darules[0]->getStrategies()[1].getName(), "strat2");

    // ASSERT_EQ(strategies[0].getName(), "strat1");
    // ASSERT_EQ(strategies[1].getName(), "strat2");
    
    std::vector<RulePtr> triggered_rules;
    std::vector<Strategy> executed_strats;
    Analyzer analyzer(darules);

    (*value_store)["bool_val"] = param_utils::getParamValue(true);

    // first tick rules triggers once --> we have exaclty one rule to sent to planning
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    // ASSERT_EQ(triggered_rules.size(), 1);

    // second tick no rules triggers, but we remind planning  --> we have exaclty one rule to sent to planning
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    // ASSERT_EQ(triggered_rules.size(), 1);

    // third tick no rules triggers, but we remind planning  --> we have exaclty one rule to sent to planning
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    // ASSERT_EQ(triggered_rules.size(), 1);
    // ASSERT_FALSE(triggered_rules[0]->getStrategies()[0].didWeTryThis());
    // ASSERT_FALSE(triggered_rules[0]->getStrategies()[1].didWeTryThis());

    executed_strats.push_back(strategy1);
    // fourth tick planning responds, we check wait if this had any effect --> we have no rule to sent to planning
    // now for three ticks (2bcs of adaptation, +1 bsc of filter pol) nothing should happen
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    // ASSERT_EQ(triggered_rules.size(), 0);
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    // ASSERT_EQ(triggered_rules.size(), 0);
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    // ASSERT_EQ(triggered_rules.size(), 0);
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);    
    // ASSERT_EQ(triggered_rules.size(), 1);
    bool mb = triggered_rules[0]->getStrategies()[1].didWeTryThis();
    // ASSERT_TRUE(triggered_rules[0]->getStrategies()[1].didWeTryThis());
    // // ASSERT_EQ(triggered_rules[0]->getStrategies()[1].getName(), "strat1");


    rclcpp::shutdown();
    return 0;
}

//int main(int argc, char** argv)
//{
//    rclcpp::init(argc, argv);
//
//    std::vector<Rule> rules = get_engel_simple_rules();
//    MAPEK_Graph systemGraph = get_engel_graph();
//    set_health_status(rules, systemGraph);
//
//    // Select strategies based on criticality level
//    Planning planning = Planning(false);
//    std::vector<Strategy> selectedStrategies = planning.selectStrategies(rules, systemGraph, Heartbeat::HB_STATUS_FAILURE);
// //    assert(selectedStrategies.size() == 1);
//    selectedStrategies = planning.selectStrategies(rules, systemGraph, Heartbeat::HB_STATUS_DEGRADED);
// //    assert(selectedStrategies.size() == 1);
//    selectedStrategies = planning.selectStrategies(rules, systemGraph, Heartbeat::HB_STATUS_OK);
// //    assert(selectedStrategies.size() == 1);
//
//    // Output selected strategies
//    std::cout << "Selected strategies:" << std::endl;
//    for (const auto& strategy : selectedStrategies) {
//        std::cout << "Strategy with success rate: " << strategy.getSuccessRate() << std::endl;
//    }
//
//    rclcpp::shutdown();
//    return 0;
//}