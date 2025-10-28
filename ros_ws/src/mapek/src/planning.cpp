#include <mapek/planning.hpp>
#include <mapek/util/logger.hpp>
#include <system_interfaces/msg/experiment_logging.hpp>
#include <system_interfaces/msg/strategy_status.hpp>

Planning::Planning()
{
    readEnvVariables(); 
    logger = BTLogger::get_global_logger();
}

void Planning::readEnvVariables()
{
    _consider_dependencies = getBooleanFromEnv("CONSIDER_DEPENDENCIES");
    _consider_criticality_level = getBooleanFromEnv("CONSIDER_CRITICALITY_LEVEL");
    _consider_cost_function = getBooleanFromEnv("CONSIDER_COST_FUNCTION");
}
 
void Planning::reset()
{
    readEnvVariables();
    affectedNodes.clear();
}

bool Planning::getBooleanFromEnv(const std::string& envVarName) {
    const char* envVarValue = std::getenv(envVarName.c_str());
    if (envVarValue == nullptr) {
        // Environment variable not set, you may define a default value if needed
        return false; // Or true, based on your default requirement
    }
    
    std::string valueStr(envVarValue);
    
    // Convert the string to lower case for consistent checks
    std::transform(valueStr.begin(), valueStr.end(), valueStr.begin(), ::tolower);
    
    // Interpret "true" and "1" as true, anything else as false
    if (valueStr == "true" || valueStr == "1") {
        return true;
    } else {
        return false;
    }
}

std::vector<Strategy> Planning::selectStrategies(const std::vector<RulePtr> &rules, const MAPEK_Graph &graph)
{
    std::vector<Strategy> strategies_failure = selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_FAILURE);
    std::vector<Strategy> strategies_degraded = selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_DEGRADED);
    std::vector<Strategy> strategies_ok = selectStrategiesBasedOnCriticalityLevel(rules, graph, Heartbeat::HB_STATUS_OK);

    strategies_failure.insert(strategies_failure.end(), strategies_degraded.begin(), strategies_degraded.end());
    strategies_failure.insert(strategies_failure.end(), strategies_ok.begin(), strategies_ok.end());
    return strategies_failure;
}

std::map<std::string, VectorGenericAdaptations> Planning::convertStrategiesToGenericAdaptations(std::vector<Strategy> strategies, ValueStorePtr value_store)
{
    std::map<std::string, std::vector<GenericAdaptation>> all_adaptations{};
    for (const auto& strategy : strategies)
    {
        auto stratey_adaptations = strategy.getAdaptations();
        for (auto genericAdaptation : stratey_adaptations)
        {
            all_adaptations[genericAdaptation.getComponent()].push_back(genericAdaptation.getAdaptation(value_store));
        }
    }
    return all_adaptations;
}

using StrategyStatus = system_interfaces::msg::StrategyStatus; // alias to generated msg constants
std::vector<Strategy> Planning::selectStrategiesBasedOnCriticalityLevel(const std::vector<RulePtr> &rules, const MAPEK_Graph &system_graph, uint8_t criticality_level)
{
    std::vector<Strategy> allStrategies;
    std::set<std::size_t> processedRules; // Tracks rules from which a strategy is already selected

    // Extract strategies across all rules
    for (const auto &rule : rules)
    {
        for (const auto &strategy : rule->getStrategies())
        {
            if (rule->getCriticalityLevel() == criticality_level)
            {
                allStrategies.push_back(strategy);
            }
        }
    }

    // Sort all strategies by their success_rate in descending order
    std::sort(allStrategies.begin(), allStrategies.end(), [this](const Strategy &a, const Strategy &b)
              { return a.getCost(_consider_cost_function) < b.getCost(_consider_cost_function); });

    std::vector<Strategy> selectedStrategies;

    for (const auto &strategy : allStrategies)
    {
        bool canSelectStrategy = true;
        if (strategy.didWeTryThis())
            continue;

        // Check if any of the nodes affected by this strategy have a dependency with a higher health status
        for (const auto &component : strategy.getAffectedComponents())
        {
            // if this component is already being adapted at the moment, we cannot select a new strategy affecting this
            if (system_graph.nodes()->at(component).busy())
            {
                canSelectStrategy = false;
                break;
            }

            if (affectedNodes.find(component) != affectedNodes.end())
            {
                // If this component is already affected by a previous strategy, skip this strategy
                canSelectStrategy = false;
                break;
            }

            if (_consider_dependencies)
            {
                // Get all dependencies for the component
                std::set<std::string> dependencies = system_graph.getDependencies(component, criticality_level);

                // If there are any dependencies with higher health status than criticality_level, skip this strategy
                for (const auto &dependency : dependencies)
                {
                    if (_consider_criticality_level)
                    {
                        if (system_graph.nodes()->at(dependency).health_status() >= criticality_level)
                        {
                            canSelectStrategy = false;
                            break;
                        }
                    }
                    else
                    {
                        // if a node we are dependent on is already going to be affected in this tick
                        // we will not select this strategy
                        if (affectedNodes.find(dependency) != affectedNodes.end())
                        {
                            canSelectStrategy = false;
                            break;
                        }
                    }

                }
            }

            if (!canSelectStrategy)
            {
                break;
            }
        }

        if (canSelectStrategy)
        {
            for (const auto &rule : rules)
            {
                auto strategies = rule->getStrategies();
                if (std::find(strategies.begin(), strategies.end(), strategy) != strategies.end())
                {
                    if (processedRules.find(rule->getHash()) == processedRules.end())
                    {
                        selectedStrategies.push_back(strategy);

                        // Strategy triggered
                        auto msg = new system_interfaces::msg::ExperimentLogging();
                        msg->timestamp = logger->get_time();
                        msg->source = "planning";
                        msg->rule_name = rule->getName();
                        msg->strategy_name = strategy.getName();
                        msg->strategy_hash = strategy.getHash();
                        msg->strategy_status = StrategyStatus::STATUS_STRATEGY_TRIGGERED;
                        logger->silent_global(msg);

                        // Mark affected nodes as processed
                        for (const auto &component : strategy.getAffectedComponents())
                        {
                            affectedNodes.insert(component);
                        }
                        processedRules.insert(rule->getHash());
                        break;
                    }
                }
            }
        }
    }

    return selectedStrategies;
}

void Planning::tick(const MAPEK_Graph &system_graph)
{
    // Remove only nodes that are currently marked as busy in the system graph from affectedNodes
    for (auto it = affectedNodes.begin(); it != affectedNodes.end(); )
    {
        auto node_iter = system_graph.nodes()->find(*it);
        if (node_iter != system_graph.nodes()->end() && node_iter->second.busy())
        {
            it = affectedNodes.erase(it);
        }
        else
        {
            ++it;
        }
    }
}
