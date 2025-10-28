#include "bt_mape_k/planning.hpp"
using AdaptationType = system_interfaces::msg::AdaptationType;
NodeStatus PlanningDecorator::tick()
{
    auto triggered_rules = getBBValue<std::vector<RulePtr>>("triggered_rules");
    auto value_store = getBBValue<ValueStorePtr>("value_store");
    auto system_state_graph = getBBValue<MAPEK_Graph>("system_state_graph");
    auto bb_adaptations = getBBValue<AdaptationMap>("adaptations");

    rclcpp::spin_some(planning->logger);
    planning->tick(system_state_graph);
    auto selected_strategies = planning->selectStrategies(triggered_rules, system_state_graph);
    auto adaptations = planning->convertStrategiesToGenericAdaptations(selected_strategies, value_store);
    // Merge bb_adaptations and adaptations
    AdaptationMap merged_adaptations = bb_adaptations;
    for (const auto& [key, vec] : adaptations) {
        auto& target_vec = merged_adaptations[key];
        target_vec.insert(target_vec.end(), vec.begin(), vec.end());
    }
    setOutput("adaptations", merged_adaptations);
    setOutput("executed_strategies", selected_strategies);
    
    auto child_status = child_node_->executeTick();
    return child_status;
}

/*
void PlanningDecorator::validateLifeCycleTransitions(std::vector<system_interfaces::msg::GenericAdaptation>& lcTransitions)
{
    if (lcTransitions.empty())
        return;
    auto bb_lc_state = getInput<int>("lc_state");
    if (!bb_lc_state)
    {
        throw RuntimeError("Error getting input port [lc_state]", bb_lc_state.error());
    }
    int current_lc_state = bb_lc_state.value();
    if (lcTransitions[0].action == system_interfaces::msg::AdaptationType::ACTION_ACTIVATE && current_lc_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        lcTransitions.erase(lcTransitions.begin());
    }
    else if (lcTransitions[0].action == system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE && current_lc_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        lcTransitions.erase(lcTransitions.begin());
    }
}

bool PlanningDecorator::areDependentNodesDegraded(std::unordered_map<std::string, std::pair<Vertex, bool>>& vertex_map, Graph g, std::string vertex_name)
{
    auto it = vertex_map.find(vertex_name);
    if (it != vertex_map.end()) {
        Vertex v = it->second.first;

        auto [adj_i, adj_end] = boost::adjacent_vertices(v, g);
        for (; adj_i != adj_end; ++adj_i) {
            Vertex adj_v = *adj_i;

            // Find which string key corresponds to the vertex descriptor adj_v
            for (auto& kv : vertex_map) {
                if (kv.second.first == adj_v) {
                    bool is_node_degraded = kv.second.second;
                    if (is_node_degraded)
                    {
                        return true;
                    }
                    break;
                }
            }
        }
        return false;
    } else {
        return false;
    } 
}
*/