#include <gtest/gtest.h>
#include <mapek/planning.hpp>
#include <mapek/util/definitions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mapek/rules/expression/ExpressionFactory.hpp>
#include <mapek/rules/RuleParser.hpp>
#include <mapek/rules/Rule.hpp>
#include <mapek/analyzer.hpp>
#include <mapek/util/parameter_message_utils.hpp>
#include "test_utils.hpp"


TEST(test_engel_analysing, single_rule_on_tick){
    rclcpp::init(0, nullptr);
    auto value_store = getValueStore();
    std::vector<RulePtr> darules;
    MAPEK_Graph graph;
    graph.addNode(GraphNode("component"));
    graph.addNode(GraphNode("component2"));

    Adaptation adaptation(
        "component", 
        system_interfaces::msg::AdaptationType::ACTION_SET_PARAMETER, 
        [](ValueStorePtr){return GenericAdaptation{};});
    Adaptation adaptation2(
        "component2", 
        system_interfaces::msg::AdaptationType::ACTION_SET_PARAMETER, 
        [](ValueStorePtr){return GenericAdaptation{};});

    // Create strategies
    Strategy strategy1({adaptation}, "strat1", 1, 0.95);
    Strategy strategy2({adaptation2}, "strat2", 2, 0.95);
    ASSERT_FALSE(strategy1.didWeTryThis());
    ASSERT_FALSE(strategy2.didWeTryThis());
    std::vector<Strategy> strategies = {strategy1, strategy2};
    auto trigger = std::make_shared<Expression>(Token("bool_val", Token::Type::VAR));
    darules.push_back(std::make_shared<Rule>("rule", trigger, strategies,Heartbeat::HB_STATUS_FAILURE, 1, Rule::FilterPolicy{1,1}, Rule::TriggerPolicy::ON_TICK));
    // ASSERT_FALSE(darules[0]->getStrategies()[0].didWeTryThis());
    // ASSERT_FALSE(darules[0]->getStrategies()[1].didWeTryThis());

    ASSERT_EQ(darules[0]->getStrategies()[0].getName(), "strat1");
    ASSERT_EQ(darules[0]->getStrategies()[1].getName(), "strat2");

    ASSERT_EQ(strategies[0].getName(), "strat1");
    ASSERT_EQ(strategies[1].getName(), "strat2");
    std::vector<RulePtr> triggered_rules;
    std::vector<Strategy> executed_strats;
    Analyzer analyzer(darules);

    (*value_store)["bool_val"] = param_utils::getParamValue(false);

    // Nothing happend --> so every thing should be fine
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    ASSERT_EQ(triggered_rules.size(), 0);
    ASSERT_EQ(graph.nodes()->at("component").health_status(), Heartbeat::HB_STATUS_OK);
    ASSERT_EQ(graph.nodes()->at("component2").health_status(), Heartbeat::HB_STATUS_OK);

    // oh no... a rule trigger 
    (*value_store)["bool_val"] = param_utils::getParamValue(true);

    // first tick rules triggers once --> we have exaclty one rule to sent to planning
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    ASSERT_EQ(triggered_rules.size(), 1);

    // assert, that every thing should be in failure state 
    ASSERT_EQ(graph.nodes()->at("component").health_status(), Heartbeat::HB_STATUS_FAILURE);
    ASSERT_EQ(graph.nodes()->at("component2").health_status(), Heartbeat::HB_STATUS_FAILURE);

    // second tick no new rules trigger, but we remind planning  --> we have exaclty one rule to sent to planning
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    ASSERT_EQ(triggered_rules.size(), 1);

    // third tick no rules triggers, but we remind planning  --> we have exaclty one rule to sent to planning
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    ASSERT_EQ(triggered_rules.size(), 1);
    ASSERT_FALSE(triggered_rules[0]->getStrategies()[0].didWeTryThis());
    ASSERT_FALSE(triggered_rules[0]->getStrategies()[1].didWeTryThis());

    // assert, we are still  in failure state 
    ASSERT_EQ(graph.nodes()->at("component").health_status(), Heartbeat::HB_STATUS_FAILURE);
    ASSERT_EQ(graph.nodes()->at("component2").health_status(), Heartbeat::HB_STATUS_FAILURE);

    executed_strats.push_back(strategy1);
    // fourth tick planning responds, we check wait if this had any effect --> we have no rule to sent to planning
    // now for three ticks (2bcs of adaptation, +1 bsc of filter pol) nothing should happen
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    ASSERT_EQ(triggered_rules.size(), 0);
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    ASSERT_EQ(triggered_rules.size(), 0);
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    ASSERT_EQ(triggered_rules.size(), 0);
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph); 
    
    // oh no the strat did not work ... anyway -...
    ASSERT_EQ(triggered_rules.size(), 1);
    ASSERT_TRUE(triggered_rules[0]->getStrategies()[0].didWeTryThis());
    ASSERT_EQ(triggered_rules[0]->getStrategies()[0].getName(), "strat1");

    // assert, we are still  in failure state 
    ASSERT_EQ(graph.nodes()->at("component").health_status(), Heartbeat::HB_STATUS_FAILURE);
    ASSERT_EQ(graph.nodes()->at("component2").health_status(), Heartbeat::HB_STATUS_FAILURE);

    // keep reminding
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph); 
    ASSERT_EQ(triggered_rules.size(), 1);

    // try the next strat
    executed_strats.push_back(strategy1);
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph); 
    // now no rule shoul be returned, as we wait for an effect
    ASSERT_EQ(triggered_rules.size(), 0);
    // yay it worked
    (*value_store)["bool_val"] = param_utils::getParamValue(false);
    
    // we need a few ticks, so that we are sure the rule is forgottem. in the mean time still no rule should be sent to planning (same as above)
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    ASSERT_EQ(triggered_rules.size(), 0);
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    ASSERT_EQ(triggered_rules.size(), 0);
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    ASSERT_EQ(triggered_rules.size(), 0);
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph); 

    // now the rule should never be trigggered again 
    for (uint i = 0; i<10; i++){
        analyzer.analyze(value_store, executed_strats, triggered_rules, graph); 
        ASSERT_EQ(triggered_rules.size(), 0);
        // assert, we are not in failure state anymore  
        ASSERT_EQ(graph.nodes()->at("component").health_status(), Heartbeat::HB_STATUS_OK);
        ASSERT_EQ(graph.nodes()->at("component2").health_status(), Heartbeat::HB_STATUS_OK);
    }
    rclcpp::shutdown();
}

TEST(test_engel_analysing, single_rule_on_change){
    rclcpp::init(0, nullptr);
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

    ASSERT_FALSE(strategy1.didWeTryThis());

    std::vector<Strategy> strategies = {strategy1};
    auto trigger = std::make_shared<Expression>(Token("bool_val", Token::Type::VAR));

    darules.push_back(std::make_shared<Rule>("rule", trigger, strategies,Heartbeat::HB_STATUS_FAILURE, 1, Rule::FilterPolicy{1,1}, Rule::TriggerPolicy::ON_CHANGE));
    ASSERT_EQ(darules[0]->getStrategies()[0].getName(), "strat1");


    ASSERT_EQ(strategies[0].getName(), "strat1");
    std::vector<RulePtr> triggered_rules;
    std::vector<Strategy> executed_strats;
    Analyzer analyzer(darules);

    (*value_store)["bool_val"] = param_utils::getParamValue(false);
    // Nothing happend --> so every thing should be fine
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    ASSERT_EQ(triggered_rules.size(), 0);
    ASSERT_EQ(graph.nodes()->at("component").health_status(), Heartbeat::HB_STATUS_OK);

    (*value_store)["bool_val"] = param_utils::getParamValue(true);

    // first tick rules triggers once --> we have exaclty one rule to sent to planning
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    ASSERT_EQ(triggered_rules.size(), 1);

    // assert, that every thing should be in failure state 
    ASSERT_EQ(graph.nodes()->at("component").health_status(), Heartbeat::HB_STATUS_FAILURE);

    // second tick no rules triggers all, but we remind planning bcs its ON_CHANGE --> we have exaclty one rule to sent to planning
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    ASSERT_EQ(triggered_rules.size(), 1);

    // assert, that every thing should be in failure state 
    ASSERT_EQ(graph.nodes()->at("component").health_status(), Heartbeat::HB_STATUS_FAILURE);

    // third tick no rules trigger, but we remind planning  --> we have exaclty one rule to sent to planning
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    ASSERT_EQ(triggered_rules.size(), 1);
    ASSERT_FALSE(triggered_rules[0]->getStrategies()[0].didWeTryThis());

    executed_strats.push_back(strategy1);
    // fourth tick planning responds, we check wait if this had any effect --> we have no rule to sent to planning
    // the rule will only trigger again if on change even disappears and reappears, so we are basically finished
    // ->> TAKE AWAY on_change Rules can only ever have one strategy 

    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    ASSERT_EQ(triggered_rules.size(), 0);
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    ASSERT_EQ(triggered_rules.size(), 0);
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    ASSERT_EQ(triggered_rules.size(), 0);
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph); 

    for (uint i = 0; i<20; i++){
        analyzer.analyze(value_store, executed_strats, triggered_rules, graph); 
        ASSERT_EQ(triggered_rules.size(), 0);
        ASSERT_EQ(graph.nodes()->at("component").health_status(), Heartbeat::HB_STATUS_OK);
    }
    rclcpp::shutdown();
}

TEST(test_engel_analysing, resolves_without_action){
    rclcpp::init(0, nullptr);
    auto value_store = getValueStore();
    std::vector<RulePtr> darules;
    MAPEK_Graph graph;
    graph.addNode(GraphNode("component"));
    graph.addNode(GraphNode("component2"));
    Adaptation adaptation(
        "component", 
        system_interfaces::msg::AdaptationType::ACTION_SET_PARAMETER, 
        [](ValueStorePtr){return GenericAdaptation{};});
    Adaptation adaptation2(
        "component2", 
        system_interfaces::msg::AdaptationType::ACTION_SET_PARAMETER, 
        [](ValueStorePtr){return GenericAdaptation{};});

    // Create strategies
    Strategy strategy1({adaptation}, "strat1", 1, 0.95);
    Strategy strategy2({adaptation2}, "strat2", 2, 0.95);
    ASSERT_FALSE(strategy1.didWeTryThis());
    ASSERT_FALSE(strategy2.didWeTryThis());
    std::vector<Strategy> strategies = {strategy1, strategy2};
    auto trigger = std::make_shared<Expression>(Token("bool_val", Token::Type::VAR));
    darules.push_back(std::make_shared<Rule>("rule", trigger, strategies,Heartbeat::HB_STATUS_FAILURE, 1, Rule::FilterPolicy{1,1}, Rule::TriggerPolicy::ON_TICK));
    // ASSERT_FALSE(darules[0]->getStrategies()[0].didWeTryThis());
    // ASSERT_FALSE(darules[0]->getStrategies()[1].didWeTryThis());

    ASSERT_EQ(darules[0]->getStrategies()[0].getName(), "strat1");
    ASSERT_EQ(darules[0]->getStrategies()[1].getName(), "strat2");

    ASSERT_EQ(strategies[0].getName(), "strat1");
    ASSERT_EQ(strategies[1].getName(), "strat2");
    std::vector<RulePtr> triggered_rules;
    std::vector<Strategy> executed_strats;
    Analyzer analyzer(darules);


    (*value_store)["bool_val"] = param_utils::getParamValue(false);

    // Nothing happend --> so every thing should be fine
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    ASSERT_EQ(triggered_rules.size(), 0);
    ASSERT_EQ(graph.nodes()->at("component").health_status(), Heartbeat::HB_STATUS_OK);
    ASSERT_EQ(graph.nodes()->at("component2").health_status(), Heartbeat::HB_STATUS_OK);

    (*value_store)["bool_val"] = param_utils::getParamValue(true);

    // first tick rules triggers once --> we have exaclty one rule to sent to planning
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    ASSERT_EQ(triggered_rules.size(), 1);

    // assert, that every thing should be in failure state 
    ASSERT_EQ(graph.nodes()->at("component").health_status(), Heartbeat::HB_STATUS_FAILURE);
    ASSERT_EQ(graph.nodes()->at("component2").health_status(), Heartbeat::HB_STATUS_FAILURE);

    // second tick no new rules trigger, but we remind planning  --> we have exaclty one rule to sent to planning
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    ASSERT_EQ(triggered_rules.size(), 1);

    // third tick no rules triggers, but we remind planning  --> we have exaclty one rule to sent to planning
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    ASSERT_EQ(triggered_rules.size(), 1);
    analyzer.analyze(value_store, executed_strats, triggered_rules, graph);
    ASSERT_EQ(triggered_rules.size(), 1);


    // For some reason the rule resolves itself
    (*value_store)["bool_val"] = param_utils::getParamValue(false);
    // we should notice and just forget the rule --> no more reminders for planning we are finished


    for (uint i = 0; i<20; i++){
        analyzer.analyze(value_store, executed_strats, triggered_rules, graph); 
        ASSERT_EQ(graph.nodes()->at("component").health_status(), Heartbeat::HB_STATUS_OK);
        ASSERT_EQ(graph.nodes()->at("component2").health_status(), Heartbeat::HB_STATUS_OK);
        ASSERT_EQ(triggered_rules.size(), 0);
    }
    rclcpp::shutdown();
}