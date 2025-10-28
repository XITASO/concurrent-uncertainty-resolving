#pragma once

#include "mapek/rules/Trigger.hpp"
#include "mapek/rules/expression/Expression.hpp"
#include "system_interfaces/msg/generic_adaptation.hpp"
#include "mapek/rules/AdaptationFactory.hpp"
#include "mapek/rules/Strategy.hpp"
#include "mapek/util/circularBuffer.hpp"

#include <functional>
#include <string>

// "Da rules are da rules and da facts are da facts" 

// Rule class: returns adaptation, if trigger condition is met
class Rule{
    public:

        struct FilterPolicy{
            uint n = 1;
            uint of = 1;
        };

        enum TriggerPolicy{
            ON_CHANGE = 0,
            ON_TICK
        };

        // Rule() {};

        bool operator==(const Rule&);

        Rule(std::string name, std::shared_ptr<Expression> trigger, std::vector<Strategy> strategies, uint criticality_level,  std::size_t hash, FilterPolicy = {1,1}, TriggerPolicy = ON_TICK);

        bool sanityCheck();

        void reset();

        // returns true if trigger condition is met
        std::optional<std::vector<Strategy>> execute(ValueStorePtr value_store) const;

        bool evaluate(ValueStorePtr value_store) const;

        std::vector<Strategy>& getStrategies() const ;

        FilterPolicy getFilterPolicy() const;
        TriggerPolicy getTriggerPolicy() const;

        // returns component
        std::set<std::string> getAffectedComponents() const;

        // checks if the component is included in the list
        bool affectsComponent(std::string component) const;

        std::string getName()const;
        std::size_t getHash()const;
        uint getCriticalityLevel()const;


    private:
        const std::string name {};
        const std::shared_ptr<Expression> trigger {};
        mutable std::vector<Strategy> strategies {};
        const uint criticality_level;
        const std::size_t hash;
        const TriggerPolicy trigger_policy;
        const FilterPolicy filter_policy;
        mutable bool is_active = false;
        mutable CircularBuffer<bool> rule_filter;
        
};

using RulePtr = std::shared_ptr<Rule>;

template <>
struct std::hash<Rule>{
    std::size_t operator()(const Rule& rule) const{
        return rule.getHash();
    }
};

template <>
struct std::hash<RulePtr>{
    std::size_t operator()(const RulePtr& rule) const{
        return rule->getHash();
    }
};