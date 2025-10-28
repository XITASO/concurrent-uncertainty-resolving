#include <gtest/gtest.h>
#include <mapek/planning.hpp>
#include <mapek/util/definitions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mapek/rules/expression/ExpressionFactory.hpp>
#include <mapek/rules/RuleParser.hpp>
#include <mapek/util/parameter_message_utils.hpp>
#include "test_utils.hpp"


TEST(test_engel_rules, test_component_getter){
    auto rules = get_engel_simple_rules2();
    auto strats1 = rules[0]->getStrategies();

    ASSERT_EQ(strats1.size(), 2);

    ASSERT_EQ(strats1[0].getAffectedComponents().size(), 2);
    ASSERT_TRUE(strats1[0].affectsComponent("camera"));
    ASSERT_TRUE(strats1[0].affectsComponent("enhancement"));

    ASSERT_EQ(strats1[1].getAffectedComponents().size(), 3);
    ASSERT_TRUE(strats1[1].affectsComponent("camera"));
    ASSERT_TRUE(strats1[1].affectsComponent("enhancement"));
    ASSERT_TRUE(strats1[1].affectsComponent("fusion"));

    ASSERT_EQ(rules[0]->getAffectedComponents().size(), 3);
    ASSERT_TRUE(rules[0]->affectsComponent("camera"));
    ASSERT_TRUE(rules[0]->affectsComponent("enhancement"));
    ASSERT_TRUE(rules[0]->affectsComponent("fusion"));
}

TEST(test_engel_rules, test_rule_filter_and_tiggering){
    auto value_store = getValueStore();
    Adaptation adaptation(
        "component", 
        0,
        [](ValueStorePtr){return GenericAdaptation{};}
    );
    // Create strategies
    Strategy strategy({adaptation}, "strat1", 1, 0.95);
    auto trigger = std::make_shared<Expression>(Token("bool_val", Token::Type::VAR));

    // no filter + on tick -> so we should trigger always if bool value is true:
    Rule rule1 ("rule_no_filter", trigger, {strategy},Heartbeat::HB_STATUS_FAILURE, 1, {1,1}, Rule::TriggerPolicy::ON_TICK);
    (*value_store)["bool_val"] = param_utils::getParamValue(true);
    ASSERT_TRUE(rule1.evaluate(value_store));
    // we should trigger multiple times in a row
    ASSERT_TRUE(rule1.evaluate(value_store));
    ASSERT_TRUE(rule1.evaluate(value_store));
    ASSERT_TRUE(rule1.evaluate(value_store));
    // we should not trigger on false, duh!
    (*value_store)["bool_val"] = param_utils::getParamValue(false);
    ASSERT_FALSE(rule1.evaluate(value_store));
    // we should trigger after off-on
    (*value_store)["bool_val"] = param_utils::getParamValue(true);
    ASSERT_TRUE(rule1.evaluate(value_store));

    // no filter + change tick -> so we should trigger only on change:
    Rule rule2("rule_no_filter", trigger, {strategy},Heartbeat::HB_STATUS_FAILURE, 1, {1,1}, Rule::TriggerPolicy::ON_CHANGE);
    (*value_store)["bool_val"] = param_utils::getParamValue(true);
    // Trigger the first time 
    ASSERT_TRUE(rule2.evaluate(value_store));
    // we should NOT trigger multiple times in a row
    ASSERT_FALSE(rule2.evaluate(value_store));
    ASSERT_FALSE(rule2.evaluate(value_store));
    ASSERT_FALSE(rule2.evaluate(value_store));
    // we should not trigger on false, duh!
    (*value_store)["bool_val"] = param_utils::getParamValue(false);
    ASSERT_FALSE(rule2.evaluate(value_store));
    // we should trigger after off-on
    (*value_store)["bool_val"] = param_utils::getParamValue(true);
    ASSERT_TRUE(rule2.evaluate(value_store));

    // filter 5 out of 7 + on_tick -> so we should trigger every time 5 trues are in the ringbuffer:
    Rule rule3("rule_no_filter", trigger, {strategy},Heartbeat::HB_STATUS_FAILURE, 1, {5,7}, Rule::TriggerPolicy::ON_TICK);
    (*value_store)["bool_val"] = param_utils::getParamValue(true);
    // Trigger the first time only 1 true; 1000000
    ASSERT_FALSE(rule3.evaluate(value_store));
    // only 2 true; 1100000
    ASSERT_FALSE(rule3.evaluate(value_store));
    // only 3 true; 1110000
    ASSERT_FALSE(rule3.evaluate(value_store));
    // only 4 true; 1111000
    ASSERT_FALSE(rule3.evaluate(value_store));
    // 5 true !! now we should trigger; 1111100
    ASSERT_TRUE(rule3.evaluate(value_store));
    // 6 true we should trigger, since we are on tick; 1111110
    ASSERT_TRUE(rule3.evaluate(value_store));
    // set false
    (*value_store)["bool_val"] = param_utils::getParamValue(false);
    // still 6/7 true, we should trigger, since we are on tick; 0111111
    ASSERT_TRUE(rule3.evaluate(value_store));
    // still 5/7 true, we should trigger, since we are on tick; 0011111
    ASSERT_TRUE(rule3.evaluate(value_store));
    // still 4/7 true, we should not trigger anymore, since we are on tick; 0001111
    ASSERT_FALSE(rule3.evaluate(value_store));

    // filter 5 out of 7 + on_change -> so we should trigger only the first time 5 trues are in the ringbuffer:
    Rule rule4("rule_no_filter", trigger, {strategy},Heartbeat::HB_STATUS_FAILURE, 1, {5,7}, Rule::TriggerPolicy::ON_CHANGE);
    (*value_store)["bool_val"] = param_utils::getParamValue(true);
    // Trigger the first time only 1 true; 1000000
    ASSERT_FALSE(rule4.evaluate(value_store));
    // only 2 true; 1100000
    ASSERT_FALSE(rule4.evaluate(value_store));
    // only 3 true; 1110000
    ASSERT_FALSE(rule4.evaluate(value_store));
    // only 4 true; 1111000
    ASSERT_FALSE(rule4.evaluate(value_store));
    // 5 true !! now we should trigger; 1111100
    ASSERT_TRUE(rule4.evaluate(value_store));
    // 6 true we should not trigger, since we are on change; 1111110
    ASSERT_FALSE(rule4.evaluate(value_store));
    // set false, slowly decrease trues inbuffer
    (*value_store)["bool_val"] = param_utils::getParamValue(false);
    // still 6/7 true, we should not trigger, since we are on change; 0111111
    ASSERT_FALSE(rule4.evaluate(value_store));
    // still 5/7 true, we should not trigger, since we are on change; 0011111
    ASSERT_FALSE(rule4.evaluate(value_store));
    // still 4/7 true, we should not trigger, 0001111
    ASSERT_FALSE(rule4.evaluate(value_store));
    // set true again
    (*value_store)["bool_val"] = param_utils::getParamValue(true);
    // still 4/7 true, we should not trigger, 1000111
    ASSERT_FALSE(rule4.evaluate(value_store));
    // still 4/7 true, we should not trigger, 1100011
    ASSERT_FALSE(rule4.evaluate(value_store));
    // still 4/7 true, we should not trigger, 1110001
    ASSERT_FALSE(rule4.evaluate(value_store));
    // still 4/7 true, we should not trigger, 1111000
    ASSERT_FALSE(rule4.evaluate(value_store));
    // still 5/7 true!!!,  we should not trigger again, 1111100
    ASSERT_TRUE(rule4.evaluate(value_store));

}

TEST(test_engel_rules, test_adaptation_factory_simple_adaptation){
    // no param adaptation
    auto value_store = getValueStore();
    AdaptationFactory af{};
    std::string simple_adapation_string = "camera action_activate";
    ASSERT_NO_THROW(auto simple_adaptation = af.produce(simple_adapation_string));
    auto simple_adaptation = af.produce(simple_adapation_string);
    ASSERT_EQ(simple_adaptation.getComponent(), "camera");
    auto e_adaptation = simple_adaptation.getAdaptation(value_store);
    auto adaptation_param = e_adaptation.parameter;
    ASSERT_EQ(e_adaptation.action, system_interfaces::msg::AdaptationType::ACTION_ACTIVATE);
}

TEST(test_engel_rules, test_adaptation_factory_param_adaptation){
    // no param adaptation
    auto value_store = getValueStore();
    AdaptationFactory af{};
    std::string simple_adapation_string = "lidar action_set_parameter my_param 1+double_val";
    ASSERT_NO_THROW(auto param_adaptation = af.produce(simple_adapation_string));
    auto param_adaptation = af.produce(simple_adapation_string);
    ASSERT_EQ(param_adaptation.getComponent(), "lidar");
    auto e_adaptation = param_adaptation.getAdaptation(value_store);
    auto adaptation_param = e_adaptation.parameter;
    ASSERT_EQ(e_adaptation.action, system_interfaces::msg::AdaptationType::ACTION_SET_PARAMETER);

    ASSERT_EQ(e_adaptation.parameter.name, "my_param");
    ASSERT_EQ(e_adaptation.parameter.value.type, ParamType::PARAMETER_DOUBLE);
    ASSERT_EQ(e_adaptation.parameter.value.double_value, 2.);

    // update value store, reevaluate adaptation
    (*value_store)["double_val"].double_value = 2.;
    e_adaptation = param_adaptation.getAdaptation(value_store);
    ASSERT_EQ(e_adaptation.parameter.value.double_value, 3.);
}

TEST(test_engel_rules, test_adaptation_factory_throws){
    // no param adaptation
    auto value_store = getValueStore();
    AdaptationFactory af{};
    
    // param on no param adaptation
    std::string faulty_adaptation = "lidar action_deactivate my_param 1+double_val";
    EXPECT_ANY_THROW(af.produce(faulty_adaptation));

    // unknown action
    faulty_adaptation = "lidar action_reboot";
    EXPECT_ANY_THROW(af.produce(faulty_adaptation));

    // no param / no component
    faulty_adaptation = "lidar action_set_parameter my_val";
    EXPECT_ANY_THROW(af.produce(faulty_adaptation));

    // wnumber args mode change
    faulty_adaptation = "lidar action_change_mode new_mode 1+43";
    EXPECT_ANY_THROW(af.produce(faulty_adaptation));

    // wrong datatype for comm change
    faulty_adaptation = "lidar action_change_communication my_val 1+43";
    auto adaptation = af.produce(faulty_adaptation);
    EXPECT_ANY_THROW(adaptation.getAdaptation(value_store));

    // wrong action for bool
    faulty_adaptation = "lidar action_increase_parameter my_val true";
    adaptation = af.produce(faulty_adaptation);
    EXPECT_ANY_THROW(adaptation.getAdaptation(value_store));

    // wrong action for string
    faulty_adaptation = "lidar action_decrease_parameter my_val \"true\"";
    adaptation = af.produce(faulty_adaptation);
    EXPECT_ANY_THROW(adaptation.getAdaptation(value_store));
}

TEST(test_engel_rules, test_rule_parsing){
    std::vector<std::shared_ptr<Rule>> rules;
    auto value_store = getValueStore();
    RuleParser parser(value_store);
    rules = parser.parse("./tests/test_rules.txt");
    ASSERT_NO_THROW(rules = parser.parse("./tests/test_rules.txt"));

    // check parsed value store values
    ASSERT_EQ(param_utils::getData<double>((*value_store)["mydouble"]), 2.);
    ASSERT_EQ(param_utils::getData<std::string>((*value_store)["mystring"]), "henlo");
    ASSERT_EQ(param_utils::getData<bool>((*value_store)["mybool"]), true);

    // check parsed rules
    ASSERT_EQ(rules.size(), 2);
    ASSERT_EQ(rules[0]->getName(), "myRule1");
    ASSERT_EQ(rules[1]->getName(), "myRule2");
    ASSERT_EQ(rules[0]->getCriticalityLevel(), Heartbeat::HB_STATUS_DEGRADED);
    ASSERT_EQ(rules[1]->getCriticalityLevel(), Heartbeat::HB_STATUS_OK);
    ASSERT_NE(rules[0]->getHash(), rules[1]->getHash());

    // check parsed Strategys
    ASSERT_EQ(rules[0]->getStrategies().size(), 2);
    ASSERT_EQ(rules[0]->getStrategies()[0].getName(), "strat_1");
    ASSERT_EQ(rules[0]->getStrategies()[1].getName(), "strat_2");
    ASSERT_EQ(rules[0]->getStrategies()[0].getAdaptations().size(), 2);
    ASSERT_EQ(rules[0]->getStrategies()[1].getAdaptations().size(), 1);

    // check hash different
    ASSERT_NE(rules[0]->getStrategies()[0].getHash(), rules[0]->getStrategies()[1].getHash());
    ASSERT_NE(rules[0]->getStrategies()[0].getHash(), rules[1]->getStrategies()[0].getHash());
    ASSERT_NE(rules[0]->getStrategies()[1].getHash(), rules[1]->getStrategies()[0].getHash());

    // implicitly check triggers
    ASSERT_TRUE(rules[0]->evaluate(value_store));
    ASSERT_FALSE(rules[1]->evaluate(value_store));

}