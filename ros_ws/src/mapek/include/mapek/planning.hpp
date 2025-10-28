#pragma once

#include <system_interfaces/msg/heartbeat.hpp>
#include <mapek/rules/Rule.hpp>
#include <mapek/util/graph.hpp>
#include <cstdlib>
#include <mapek/util/logger.hpp>
#include <mapek/rules/Adaptation.hpp>
#include <mapek/util/definitions.hpp>

class Planning {
public:
    Planning();
    void tick(const MAPEK_Graph& system_graph);

    std::vector<Strategy> selectStrategies(const std::vector<RulePtr>& rules, const MAPEK_Graph& system_graph);
    std::map<std::string, VectorGenericAdaptations> convertStrategiesToGenericAdaptations(std::vector<Strategy> strategies, ValueStorePtr value_store);
    std::vector<Strategy> selectStrategiesBasedOnCriticalityLevel(const std::vector<RulePtr>& rules, const MAPEK_Graph& system_graph, uint8_t criticality_level);
    const bool getConsiderDependencies() {return _consider_dependencies;}
    const bool getConsiderCriticalityLevel() {return _consider_criticality_level;}
    const bool getConsiderCostFunction() {return _consider_cost_function;}
    void reset();
    std::shared_ptr<BTLogger> logger;
private:
    bool _consider_dependencies;
    bool _consider_criticality_level;
    bool _consider_cost_function; // if true, use the cost function of a strategy to prioritize, if false use the success probability
    std::set<std::string> affectedNodes; // Tracks nodes that are already affected by a strategy

    void readEnvVariables();
    bool getBooleanFromEnv(const std::string& envVarName);
}; 