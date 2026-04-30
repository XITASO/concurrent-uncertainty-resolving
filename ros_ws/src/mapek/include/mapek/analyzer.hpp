#pragma once
#include <mapek/util/definitions.hpp>
#include <mapek/rules/Rule.hpp>
#include <unordered_set>
#include <mapek/util/logger.hpp>
#include <unordered_map>
#include <mapek/util/graph.hpp>

class Analyzer{
    public:
        // if rules path is given, i.e. not "" we update probabilities to file
        Analyzer(std::vector<RulePtr> rules, std::string rule_file = "");

        std::shared_ptr<BTLogger> logger;
        void analyze(
            ValueStorePtr value_store,
            std::vector<Strategy>& executed_strategies,
            std::vector<RulePtr>& triggered_rules,
            MAPEK_Graph &graph
        );

    private:
        std::string rule_file;
        bool update_success_rates_and_probabilities;

        std::vector<RulePtr> rules;
        std::unordered_set<RulePtr> rules_sent_to_planning;
        std::unordered_map<RulePtr, int> adapted_rules;
        std::unordered_map<RulePtr, Strategy*> last_executed_strategy;
};