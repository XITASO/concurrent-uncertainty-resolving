#pragma once

#include <mapek/rules/Rule.hpp>
#include <mapek/util/graph.hpp>
#include <mapek/util/definitions.hpp>

struct ENGELRuleConfig
{
    bool bad_segmentation = false;
    bool depth_dead = false;
    bool fusion_dead = false;
    bool segmentation_dead = false;
    bool auto_focus_needed = false;
};

MAPEK_Graph get_simple_graph();
std::vector<RulePtr> get_simple_rules();
MAPEK_Graph get_engel_graph();
std::vector<RulePtr> get_engel_simple_rules();
std::vector<RulePtr> get_engel_simple_rules2();
void set_health_status(const std::vector<RulePtr> rules, MAPEK_Graph &graph);
ValueStorePtr getValueStore();
rcl_interfaces::msg::Parameter createStringParameterObject(std::string name, std::string value);

std::vector<RulePtr> get_engel_configurable_rules(ENGELRuleConfig config);