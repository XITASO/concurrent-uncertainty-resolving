#include "behaviortree_cpp/bt_factory.h"
#include "bt_mape_k/json.hpp"
#include <fstream>
#include <mapek/util/graph.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <mapek/util/definitions.hpp>
#include <mapek/rules/Strategy.hpp>


#include <string>


using namespace BT;

class InitializeBlackboard: public SyncActionNode
{
public:
    InitializeBlackboard(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config){
            random_value_blackboard = std::make_shared<ValueStore>();
            setOutput("value_store", random_value_blackboard);
        }

    static PortsList providedPorts()
    {
        return {
            OutputPort<std::vector<Strategy>>("executed_strategies"),
            OutputPort<AdaptationMap>("adaptations"),
            OutputPort<ValueStorePtr>("value_store")
        };
    }

    NodeStatus tick() override;

    private:
        ValueStorePtr random_value_blackboard;
        std::vector<Strategy> executed_strategies{};
        AdaptationMap adaptations{};

};