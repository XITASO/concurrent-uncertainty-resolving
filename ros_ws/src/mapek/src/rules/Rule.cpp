#include "mapek/rules/Rule.hpp"
#include <iostream>
#include <optional>

/**
 * @brief Constructor of the Rule class
 * 
 * @param name The name of the rule
 * @param components All components the rule applies to
 * @param trigger The trigger to be checked
 * @param adaptation The adaptation top be execute if triggered
 * @param on_change execution policy: if true execute the adaptation only if the trigger state changes,
 * if false execute on every trigger
 * 
 */
Rule::Rule(std::string name, std::shared_ptr<Expression> trigger, std::vector<Strategy> strategies, uint criticality_level, std::size_t hash, FilterPolicy filter_policy, TriggerPolicy trigger_policy):
    trigger(trigger),
    strategies(strategies),
    criticality_level(criticality_level),
    hash(hash),
    name(name),
    trigger_policy(trigger_policy),
    filter_policy(filter_policy),
    rule_filter(CircularBuffer<bool>(filter_policy.of))
    {
        rule_filter.fill(false);
    }

bool Rule::operator==(const Rule& other){
    return other.getHash() == this->getHash();
}

/**
 * @brief Executes the rule
 * 
 * Checks the trigger and if is evauates to "true", it returns the corresponding adaptation
 * 
 * @return Optional Adaptation contains the adaptation if triggered
 */
std::optional<std::vector<Strategy>> Rule::execute(ValueStorePtr value_store) const{
    if (evaluate(value_store))
        return strategies;
    return{};
}

bool Rule::sanityCheck(){
    double summed_success {};
    for (const auto& strategy : strategies){
       summed_success += strategy.getSuccessRate();
    }
    if (std::abs(summed_success - 100.) > 0.1)
        return false;

    if (filter_policy.n > filter_policy.of)
        return false;

    return true;
}

void Rule::reset(){
    for (auto & strat : strategies ){
        strat.reset();
    }
}

Rule::FilterPolicy Rule::getFilterPolicy() const{
    return filter_policy;
}

Rule::TriggerPolicy Rule::getTriggerPolicy() const{
    return trigger_policy;
}

bool Rule::evaluate(ValueStorePtr value_store) const {
    auto result = trigger->evaluate(value_store);
    if (result.getType(value_store) != Token::Type::BOOL)
        throw std::runtime_error("trigger does not evaluate to bool");

    if (result.getValue<bool>(value_store)) {
        rule_filter.put(true);
    }else{
        rule_filter.put(false);
    }

    if(rule_filter.count(true) >= filter_policy.n){
        if(!is_active || trigger_policy == TriggerPolicy::ON_TICK) {
            is_active = true;
            return true;
        } 
    }else{
        is_active = false;
    }
    return false;
}

// careful this bad boy returns Strats by reference
std::vector<Strategy>& Rule::getStrategies() const {return strategies;}

/**
 * @brief Getter for the rule name
 * 
 * @return string The name of the rule
 */
std::string Rule::getName()const{
    return name;
}

std::size_t Rule::getHash()const{
    return hash;
}

uint Rule::getCriticalityLevel()const{
    return criticality_level; 
}

/**
 * @brief Executes the rule
 * 
 * Checks the trigger and if is evauates to "true" AND the rule applied to the given component, it returns the corresponding adaptation 
 * 
 * @param component the name of the component to filter by
 * 
 * @return Optional Adaptation contains the adaptation if triggered
 */
// std::optional<GenericAdaptation> Rule::execute(ValueStorePtr value_store, std::string component) const{
//     if(affectsComponent(component)){
//         return execute(value_store);
//     }
//     return{};
// }

bool Rule::affectsComponent(std::string query) const{
    for (const auto& strategy : strategies){
        if (strategy.affectsComponent(query))
            return true;
    }
    return false;
}

/**
 * @brief Getter for the rules components
 * 
 * @return vector of the components the rules applies to
 */
std::set<std::string> Rule::getAffectedComponents() const{
    std::set<std::string> affected_components;
    for (const auto& strategy : strategies){
        auto strategy_components = strategy.getAffectedComponents();
        affected_components.insert(strategy_components.begin(), strategy_components.end() );
    }
    return affected_components;
}