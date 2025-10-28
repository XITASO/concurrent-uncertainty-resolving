#pragma once
#include <mapek/util/definitions.hpp>
#include <mapek/rules/Rule.hpp>
#include <unordered_set>
#include <mapek/util/logger.hpp>
#include <unordered_map>
#include <mapek/util/graph.hpp>

class Analyzer{
    public:
        Analyzer(std::vector<RulePtr> rules);

        std::shared_ptr<BTLogger> logger;
        void analyze(
            ValueStorePtr value_store,
            std::vector<Strategy>& executed_strategies,
            std::vector<RulePtr>& triggered_rules,
            MAPEK_Graph &graph
        );

    private:
        std::vector<RulePtr> rules;
        std::unordered_set<RulePtr> rules_sent_to_planning;
        std::unordered_map<RulePtr, int> adapted_rules;
        std::unordered_map<RulePtr, Strategy> last_executed_strategy;
};