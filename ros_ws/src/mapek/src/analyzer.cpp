#include <mapek/analyzer.hpp>
#include <iostream>
#include <mapek/util/logger.hpp>
#include <system_interfaces/msg/experiment_logging.hpp>
#include <system_interfaces/msg/strategy_status.hpp>

void setHealthStatus(const RulePtr rule, MAPEK_Graph &graph){
    for (const auto &strategy : rule->getStrategies()){
        for (const std::string &component : strategy.getAffectedComponents())
        {
            graph[component].health_status(rule->getCriticalityLevel());
        }
    }
}

void resetHealthStatus(const RulePtr rule, MAPEK_Graph &graph){
    for (const auto &strategy : rule->getStrategies()){
        for (const std::string &component : strategy.getAffectedComponents())
        {
            graph[component].health_status(Heartbeat::HB_STATUS_OK);
        }
    }
}

Analyzer::Analyzer(std::vector<RulePtr> rules) : rules(rules){
    logger = BTLogger::get_global_logger();
}

using StrategyStatus = system_interfaces::msg::StrategyStatus;
void Analyzer::analyze(
    ValueStorePtr value_store ,
    std::vector<Strategy>& executed_strategies,
    std::vector<RulePtr>& triggered_rules,
    MAPEK_Graph &graph){

    //1. handle rules that are currently adapted
    // increase counter to count ticks since adaptation
    for (auto &rule_ctr : adapted_rules){
        rule_ctr.second += 1;
    }


    //2. loop over all existing rules, evaluate and decide what to de depending on wether the rule is currently processed
    // process tried _strats
    for (const auto & strat : executed_strategies){
        //find corresponding rule and strat
        bool found_rule = false; 
        for(const auto & rule :rules_sent_to_planning){
            for (auto & rule_strat : rule->getStrategies()){
                if (strat == rule_strat){
                    // we found our rule: noice!
                    // mark strat OF RULE as marked
                    rule_strat.markAsTried();
                    found_rule = true;
                    last_executed_strategy[rule] = strat;
                    break;
                }  
            }
            if (found_rule){
                // check that the rule is not already being adapted
                if (adapted_rules.count(rule)){
                    throw std::runtime_error("[Analysing] Something went very wrong !double adaptation!");
                }
                // move rule to adapted_rules
                adapted_rules[rule] = -strat.getExecutionEstimate();

                // since we break here, we can erase without doing iterator magic, but handle all the action befor deleting
                rules_sent_to_planning.erase(rule);
                break;
            }
        }
        // if I get an unexpected strategy, I WILL panic
        if (!found_rule){
            throw std::runtime_error("[Analysing] Something went very wrong !adaptating unknown rule!");
        }
    }

    executed_strategies.clear();
    triggered_rules.clear();

    //3. loop over all strats, find corresponding rule and mark it, throw if smth is wrong
    for (auto & rule : rules){
        if (rule->evaluate(value_store)){


            // if this is new it is not in the adapted list 
            // --> reset and add to triggered adaptations
            // mark that we sent the rule to planning
            // were done
            if (!adapted_rules.count(rule) && !rules_sent_to_planning.count(rule)){
                // since we work with pointers now this should work as each rule only exists once (should)
                rule->reset();
                rules_sent_to_planning.emplace(rule);

                // this is the first time the rule triggers: set criticality
                setHealthStatus(rule, graph);


                // Log Rule triggered
                auto msg = new system_interfaces::msg::ExperimentLogging();
                msg->timestamp = logger->get_time(); 
                msg->source = "analyzer" ;  
                msg->rule_name = rule->getName();
                logger->silent_global(msg);
                continue;
            }

            // if this rule is known, we check if we expectd the change to take place
            // move the rule from adapted rules to rules sent to planning
            if (adapted_rules.count(rule) && (adapted_rules[rule] > (int)rule->getFilterPolicy().n)){

                auto msg = new system_interfaces::msg::ExperimentLogging();
                msg->timestamp = logger->get_time(); 
                msg->source = "analyzer" ;  
                msg->rule_name = rule->getName();
                msg->success = false;
                if (last_executed_strategy.count(rule)){
                    msg->strategy_name = last_executed_strategy[rule].getName();
                    msg->strategy_hash = last_executed_strategy[rule].getHash();
                }
                msg->strategy_status = StrategyStatus::STATUS_STRATEGY_FINISHED_UNSUCCESSFUL;
                logger->silent_global(msg);

                rules_sent_to_planning.emplace(rule);
                adapted_rules.erase(rule);
                continue;
            }

            // I think that was it: the remaining case is a rule that was sent to planning but not yet adapted, 
            // here we do nothing on trigger, those are handeled later

        } else {
            // we only even care if some adaptation was done 
            if (adapted_rules.count(rule)){
                // also we wait until we expect to fix to be done. No harm in that.
                // --> erase rule. We did it!
                
                if (adapted_rules[rule] >= rule->getFilterPolicy().n){
                    adapted_rules.erase(rule);
                    resetHealthStatus(rule, graph);
                    
                    // Log that we did it
                    auto msg = new system_interfaces::msg::ExperimentLogging();
                    msg->timestamp = logger->get_time(); 
                    msg->source = "analyzer" ;  
                    msg->rule_name = rule->getName();
                    msg->success = true;
                    if (last_executed_strategy.count(rule)){
                        msg->strategy_name = last_executed_strategy[rule].getName();
                        msg->strategy_hash = last_executed_strategy[rule].getHash();
                    }
                    msg->strategy_status = StrategyStatus::STATUS_STRATEGY_FINISHED_SUCCESSFUL;
                    logger->silent_global(msg);
                }
            }
            // if the rule is on Tick and has resolved itself --> Perfect
            if (rules_sent_to_planning.count(rule)){

                if (rule->getTriggerPolicy() == Rule::TriggerPolicy::ON_TICK){
                    rules_sent_to_planning.erase(rule);
                    resetHealthStatus(rule, graph);

                    // Log that every thing is fine we did it
                    auto msg = new system_interfaces::msg::ExperimentLogging();
                    msg->timestamp = logger->get_time(); 
                    msg->source = "analyzer" ;  
                    msg->rule_name = rule->getName();
                    msg->success = true;
                    if (last_executed_strategy.count(rule)){
                        msg->strategy_name = last_executed_strategy[rule].getName();
                        msg->strategy_hash = last_executed_strategy[rule].getHash();
                    }
                    msg->strategy_status = StrategyStatus::STATUS_NO_STRATEGY_SUCCESSFUL;
                    logger->silent_global(msg);
                }
            }
        }
    }

    // Send all rules that are now in rules_sent_to_planning to planning
    // those are the newly added ones, and also the old ones to remind planning
    for(const auto & rule :rules_sent_to_planning){
        triggered_rules.push_back(rule);
    }

}
