#include "test_utils.hpp"
#include <mapek/util/parameter_message_utils.hpp>

ValueStorePtr getValueStore()
{
    auto value_store = std::make_shared<ValueStore>();
    (*value_store)["double_val"] = param_utils::getParamValue(1.0);
    (*value_store)["string_val"] = param_utils::getParamValue("hello");
    (*value_store)["bool_val"] = param_utils::getParamValue(true);
    return value_store;
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

std::vector<RulePtr> get_simple_rules()
{

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
    Adaptation adaptation3(
        "C",
        system_interfaces::msg::AdaptationType::ACTION_CHANGE_MODE,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_CHANGE_MODE;
            return ga;
        });
    Adaptation adaptation4(
        "C",
        system_interfaces::msg::AdaptationType::ACTION_CHANGE_COMMUNICATION,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_CHANGE_COMMUNICATION;
            return ga;
        });

    // Create strategies
    Strategy strategy1({adaptation1}, "s1", 1, 0.95);
    Strategy strategy2({adaptation2}, "s2", 2, 0.90);
    Strategy strategy3({adaptation3}, "s3", 3, 0.85);
    Strategy strategy4({adaptation4}, "s4", 4, 0.94);

    auto trigger = std::make_shared<Expression>(Token("true", Token::Type::BOOL));

    // Create MAPEK rules
    RulePtr rule1 = std::make_shared<Rule>(Rule("r1", trigger, {strategy1, strategy4}, Heartbeat::HB_STATUS_FAILURE, 1));
    RulePtr rule2 = std::make_shared<Rule>(Rule("r2", trigger, {strategy2}, Heartbeat::HB_STATUS_DEGRADED, 2));
    RulePtr rule3 = std::make_shared<Rule>(Rule("r3", trigger, {strategy3}, Heartbeat::HB_STATUS_FAILURE, 3));

    std::vector<RulePtr> rules = {rule1, rule2, rule3};
    return rules;
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

std::vector<RulePtr> get_engel_simple_rules()
{
    Adaptation adaptation1(
        "camera",
        system_interfaces::msg::AdaptationType::ACTION_ACTIVATE,
        [](ValueStorePtr)
        {
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_ACTIVATE;
            return ga;
        });
    Adaptation adaptation2(
        "enhancement",
        system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE;
            return ga;
        });
    Adaptation adaptation3(
        "fusion",
        system_interfaces::msg::AdaptationType::ACTION_CHANGE_MODE,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_CHANGE_MODE;
            return ga;
        });
    Adaptation adaptation4(
        "depth",
        system_interfaces::msg::AdaptationType::ACTION_CHANGE_COMMUNICATION,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_CHANGE_COMMUNICATION;
            return ga;
        });
    // Create strategies
    Strategy strategy1({adaptation1}, "s1", 1, 0.95);
    Strategy strategy2({adaptation2}, "s2", 2, 0.90);
    Strategy strategy3({adaptation3}, "s3", 3, 0.85);
    Strategy strategy4({adaptation4}, "s4", 4, 0.94);

    auto trigger = std::make_shared<Expression>(Token("true", Token::Type::BOOL));
    std::shared_ptr<Rule> rule1 = std::make_shared<Rule>(Rule("bad_camera_data", trigger, {strategy1, strategy2}, Heartbeat::HB_STATUS_DEGRADED, 1));
    std::shared_ptr<Rule> rule2 = std::make_shared<Rule>(Rule("no_fusion_results", trigger, {strategy3}, Heartbeat::HB_STATUS_FAILURE, 2));
    std::shared_ptr<Rule> rule3 = std::make_shared<Rule>(Rule("add_depth", trigger, {strategy4}, Heartbeat::HB_STATUS_OK, 3));
    std::vector<RulePtr> rules = {rule1, rule2, rule3};
    return rules;
}
std::vector<RulePtr> get_engel_simple_rules2()
{
    Adaptation adaptation1(
        "camera",
        system_interfaces::msg::AdaptationType::ACTION_ACTIVATE,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_ACTIVATE;
            return ga;
        });
    Adaptation adaptation2(
        "enhancement",
        system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE;
            return ga;
        });
    Adaptation adaptation3(
        "fusion",
        system_interfaces::msg::AdaptationType::ACTION_CHANGE_MODE,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_CHANGE_MODE;
            return ga;
        });
    Adaptation adaptation4(
        "depth",
        system_interfaces::msg::AdaptationType::ACTION_CHANGE_COMMUNICATION,
        [](ValueStorePtr){
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_CHANGE_COMMUNICATION;
            return ga;
        });
    // Create strategies
    Strategy strategy1({adaptation1, adaptation2}, "s1", 1, 0.95);
    Strategy strategy2({adaptation1, adaptation2, adaptation3}, "s2", 2, 0.90);
    Strategy strategy3({adaptation3}, "s3", 3, 0.85);
    Strategy strategy4({adaptation4, adaptation3}, "s4", 4, 0.94);

    auto trigger = std::make_shared<Expression>(Token("true", Token::Type::BOOL));
    RulePtr rule1 = std::make_shared<Rule>(Rule("bad_camera_data", trigger, {strategy1, strategy2}, Heartbeat::HB_STATUS_FAILURE, 1));
    RulePtr rule2 = std::make_shared<Rule>(Rule("no_fusion_results", trigger, {strategy1, strategy2}, Heartbeat::HB_STATUS_DEGRADED, 2));
    RulePtr rule3 = std::make_shared<Rule>(Rule("add_depth", trigger, {strategy3}, Heartbeat::HB_STATUS_FAILURE, 3));
    std::vector<RulePtr> rules = {rule1, rule2, rule3};
    return rules;
}

rcl_interfaces::msg::Parameter createStringParameterObject(std::string name, std::string value)
{
    // Create an instance of the Parameter message
    rcl_interfaces::msg::Parameter parameter;

    // Set the name of the parameter
    parameter.name = name;

    // Create an instance of the ParameterValue message and set it as a string
    rcl_interfaces::msg::ParameterValue parameter_value;
    parameter_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    parameter_value.string_value = value;

    // Assign the ParameterValue to our Parameter instance
    parameter.value = parameter_value;
    return parameter;
}

std::vector<RulePtr> get_engel_configurable_rules(ENGELRuleConfig config)
{
    Adaptation adapt_enhancement_activate(
        "enhancement",
        system_interfaces::msg::AdaptationType::ACTION_ACTIVATE,
        [](ValueStorePtr)
        {
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_ACTIVATE;
            return ga;
        });

    Adaptation adapt_fusion_get_enhanced(
        "fusion",
        system_interfaces::msg::AdaptationType::ACTION_CHANGE_COMMUNICATION,
        [](ValueStorePtr)
        {
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_CHANGE_COMMUNICATION;
            ga.parameter = createStringParameterObject("rgb_topic", "rgb_enhanced");
            return ga;
        });

    Adaptation adapt_fusion_recalibrate(
        "fusion",
        system_interfaces::msg::AdaptationType::ACTION_SET_PARAMETER,
        [](ValueStorePtr)
        {
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_SET_PARAMETER;
            ga.parameter = createStringParameterObject("do_recalib", "True");
            return ga;
        });

    Adaptation adapt_fusion_get_raw(
        "fusion",
        system_interfaces::msg::AdaptationType::ACTION_CHANGE_COMMUNICATION,
        [](ValueStorePtr)
        {
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_CHANGE_COMMUNICATION;
            ga.parameter = createStringParameterObject("rgb_topic", "rgb_raw");
            return ga;
        });

    Adaptation adapt_enhancement_deactivate(
        "enhancement",
        system_interfaces::msg::AdaptationType::ACTION_ACTIVATE,
        [](ValueStorePtr)
        {
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_ACTIVATE;
            return ga;
        });

    Adaptation adapt_fusion_restart(
        "fusion",
        system_interfaces::msg::AdaptationType::ACTION_RESTART,
        [](ValueStorePtr)
        {
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_RESTART;
            return ga;
        });

    Adaptation adapt_fusion_redeploy(
        "fusion",
        system_interfaces::msg::AdaptationType::ACTION_REDEPLOY,
        [](ValueStorePtr)
        {
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_REDEPLOY;
            return ga;
        });

    Adaptation adapt_segmentation_restart(
        "segmentation",
        system_interfaces::msg::AdaptationType::ACTION_RESTART,
        [](ValueStorePtr)
        {
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_RESTART;
            return ga;
        });

    Adaptation adapt_segmentation_redeploy(
        "segmentation",
        system_interfaces::msg::AdaptationType::ACTION_REDEPLOY,
        [](ValueStorePtr)
        {
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_REDEPLOY;
            return ga;
        });

    Adaptation adapt_depth_restart(
        "depth",
        system_interfaces::msg::AdaptationType::ACTION_RESTART,
        [](ValueStorePtr)
        {
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_RESTART;
            return ga;
        });

    Adaptation adapt_depth_redeploy(
        "depth",
        system_interfaces::msg::AdaptationType::ACTION_REDEPLOY,
        [](ValueStorePtr)
        {
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_REDEPLOY;
            return ga;
        });

    Adaptation adapt_autofocus(
        "camera",
        system_interfaces::msg::AdaptationType::ACTION_SET_PARAMETER,
        [](ValueStorePtr)
        {
            GenericAdaptation ga;
            ga.action = system_interfaces::msg::AdaptationType::ACTION_SET_PARAMETER;
            ga.parameter = createStringParameterObject("do_autofocus", "True");
            return ga;
        });

    Strategy strat_activate_enhancement({adapt_enhancement_activate, adapt_fusion_get_enhanced}, "s1", 1, 0.95);
    Strategy strat_deactivate_enhancement({adapt_enhancement_deactivate, adapt_fusion_get_raw}, "s1", 1, 0.95);
    Strategy strat_recalib_fusion({adapt_fusion_recalibrate}, "s1", 1, 0.85);

    Strategy strat_restart_depth({adapt_depth_restart}, "s2", 2, 0.90);
    Strategy strat_redeploy_depth({adapt_depth_redeploy}, "s2", 2, 0.90);

    Strategy strat_restart_fusion({adapt_fusion_restart}, "s2", 2, 0.90);
    Strategy strat_redeploy_fusion({adapt_fusion_redeploy}, "s2", 2, 0.90);

    Strategy strat_restart_segmentation({adapt_segmentation_restart}, "s2", 2, 0.90);
    Strategy strat_redeploy_segmentation({adapt_segmentation_redeploy}, "s2", 2, 0.90);

    Strategy strat_autofocus({adapt_autofocus}, "s4", 4, 1.0);

    auto trigger = std::make_shared<Expression>(Token("true", Token::Type::BOOL));

    RulePtr rule_bad_seg = std::make_shared<Rule>(Rule("segmentation_bad", trigger, {strat_activate_enhancement, strat_deactivate_enhancement, strat_recalib_fusion}, Heartbeat::HB_STATUS_DEGRADED, 1));
    RulePtr rule_depth_dead = std::make_shared<Rule>(Rule("depth_dead", trigger, {strat_redeploy_depth, strat_restart_depth}, Heartbeat::HB_STATUS_FAILURE, 2));
    RulePtr rule_fusion_dead = std::make_shared<Rule>(Rule("fusion_dead", trigger, {strat_redeploy_fusion, strat_restart_fusion}, Heartbeat::HB_STATUS_FAILURE, 3));
    RulePtr rule_seg_dead = std::make_shared<Rule>(Rule("segmentation_dead", trigger, {strat_redeploy_segmentation, strat_restart_segmentation}, Heartbeat::HB_STATUS_FAILURE, 4));
    RulePtr rule_autofocus = std::make_shared<Rule>(Rule("auto_focus_needed", trigger, {strat_autofocus}, Heartbeat::HB_STATUS_OK, 5));
    std::vector<RulePtr> rules = {};
    if (config.bad_segmentation)
        rules.push_back(rule_bad_seg);
    if (config.depth_dead)
        rules.push_back(rule_depth_dead);
    if (config.fusion_dead)
        rules.push_back(rule_fusion_dead);
    if (config.segmentation_dead)
        rules.push_back(rule_seg_dead);
    if (config.auto_focus_needed)
        rules.push_back(rule_autofocus);

    return rules;
}

void set_health_status(const std::vector<RulePtr> rules, MAPEK_Graph &graph)
{
    for (const auto &rule : rules)
    {
        for (const auto &strategy : rule->getStrategies())
        {
            for (const std::string &component : strategy.getAffectedComponents())
            {
                graph[component].health_status(rule->getCriticalityLevel());
            }
        }
    }
}