#pragma once

#include <string>
#include <unordered_map>
#include <map>

#include "mapek/rules/Rule.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "mapek/rules/AdaptationFactory.hpp"
#include "mapek/rules/expression/ExpressionFactory.hpp"


// helper class to handle rule file
class RuleParser{
    public:
        RuleParser(ValueStorePtr);
        // parses the file to a list of rules and sets constants
        std::vector<std::shared_ptr<Rule>> parse(std::string path) const;
    private:
        // parses first line of rule, rule name and activation type
        void parse_header(std::string line, std::string & name) const;
        
        // parses second line of rule, Policies
        void parse_policies(std::string line, uint& criticality_level,  Rule::TriggerPolicy& policy, Rule::FilterPolicy& fitler) const;

        // parses third line of rule, trigger
        std::shared_ptr<Expression> parse_triggers(std::string) const;

        // parses fourth line of rule, adaptation by using AdaptationGenerator factory
        std::vector<Strategy> parse_strategies(std::ifstream*, const std::string & rule_name) const;
        Strategy parse_strategy(std::ifstream*, const std::string & rule_name) const;

        // parses single adaptations Adaptation factory
        Adaptation parse_adaptation(std::string) const;

        // parses constant and writes to value_store
        void log_constant(std::string) const;

        ExpressionFactory expression_factory{};
        AdaptationFactory adaptation_factory{};
        ValueStorePtr value_store {};
        mutable std::string current_rule_all_string {};
        std::hash<std::string> hasher{};
        
};