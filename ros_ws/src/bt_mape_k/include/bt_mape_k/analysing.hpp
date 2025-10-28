#include "behaviortree_cpp/bt_factory.h"

#include "system_interfaces/msg/generic_adaptation.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "system_interfaces/msg/comm_change_input.hpp"
#include "system_interfaces/msg/parametrization_input.hpp"
#include <mapek/rules/Rule.hpp>
#include <mapek/rules/RuleParser.hpp>
#include "mapek/util/adaptation_utils.hpp"
#include <mapek/util/logger.hpp>
#include <mapek/analyzer.hpp>
#include <mapek/util/definitions.hpp>
#include <mapek/util/circularBuffer.hpp>
#include <unordered_map>
#include <mapek/util/graph.hpp>


using namespace BT;

class AnalysisDecorator : public DecoratorNode
{
public:
    AnalysisDecorator(const std::string& name, const NodeConfiguration& config, std::string rules_path);

    static PortsList providedPorts()
    {
        return {
            InputPort<ValueStorePtr>("value_store"),
            InputPort<std::vector<Strategy>>("executed_strategies"),
            OutputPort<std::vector<RulePtr>>("triggered_rules"),
            BidirectionalPort<MAPEK_Graph>("system_state_graph"),
        };
    }

    NodeStatus tick() override;

    private: 
        std::shared_ptr<Analyzer> analyzer;
        std::shared_ptr<BTLogger> experiment_logger;
        system_interfaces::msg::ExperimentLogging message;

        template<typename T>
        T getBBValue(std::string key){
            auto bb_val = getInput<T>(key);
            if (!bb_val){
                throw std::runtime_error("Analysing: Error getting input port ["+key+"]");
            }
            return bb_val.value();

        }

};


/*
// TODO do we want / need this? it does not to be home in Analysis anymore
std::vector<system_interfaces::msg::GenericAdaptation> evaluateFailedAdaptations(std::vector<system_interfaces::msg::GenericAdaptation> failed_adaptations, uint max_failures);
std::vector<system_interfaces::msg::GenericAdaptation> spiceIncomingAdaptationsWithFailedOnes(std::vector<system_interfaces::msg::GenericAdaptation> new_adaptations, std::vector<system_interfaces::msg::GenericAdaptation> retry_adaptations);
void filterInsaneAdaptations(std::vector<system_interfaces::msg::GenericAdaptation>& adaptations);        
void sortAndWriteAdaptations(std::vector<system_interfaces::msg::GenericAdaptation>);
        std::vector<std::shared_ptr<Rule>> rules;

*/