#include "behaviortree_cpp/bt_factory.h"

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "system_interfaces/msg/heartbeat.hpp"
#include "system_interfaces/msg/comm_change_input.hpp"
#include "system_interfaces/msg/generic_adaptation.hpp"
#include "system_interfaces/msg/adaptation_type.hpp"
#include <system_interfaces/msg/experiment_logging.hpp>
#include "mapek/util/logger.hpp"
#include <mapek/util/graph.hpp>
#include "mapek/util/adaptation_utils.hpp"
#include <mapek/rules/Rule.hpp>
#include <mapek/rules/Adaptation.hpp>
#include <mapek/planning.hpp>
using namespace BT;

// Define a graph using an adjacency list
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

class PlanningDecorator: public DecoratorNode
{
public:    
    PlanningDecorator(const std::string& name, const NodeConfiguration& config)
        : DecoratorNode(name, config) {
            experiment_logger = BTLogger::get_global_logger();
            message.source = "planning";
            planning = std::make_shared<Planning>();
        }

    static PortsList providedPorts()
    {
        return { 
            OutputPort<std::vector<Strategy>>("executed_strategies"),
            InputPort<std::vector<RulePtr>>("triggered_rules"),
            InputPort<MAPEK_Graph>("system_state_graph"),
            InputPort<ValueStorePtr>("value_store"),
            BidirectionalPort<AdaptationMap>("adaptations"),
        };
    }

    template<typename T>
    T getBBValue(std::string key){
        auto bb_val = getInput<T>(key);
        if (!bb_val){
            throw std::runtime_error("Planning: Error getting input port ["+key+"]");
        }
        return bb_val.value();

    }
    NodeStatus tick() override;
    std::shared_ptr<BTLogger> experiment_logger;
    
private:
    // the timestamp when this node was last in a degraded or failure state
    system_interfaces::msg::ExperimentLogging message;
    std::shared_ptr<Planning> planning;
};
