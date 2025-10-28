#include <bt_mape_k/initializing.hpp>

/**
 * @brief Implements the behavior tree node's tick action to initialize the blackboard with data from a JSON file
 *        and build a directed graph representing component dependencies.
 * 
 * @return NodeStatus indicating the execution result, returns NodeStatus::SUCCESS if initialization completes
 *         successfully, or NodeStatus::FAILURE if file access encounters an error.
 * 
 * The tick function reads data from a specified JSON file, parsing it to set initial values on the blackboard
 * (e.g., strings, integers, floats, booleans). It constructs a system state graph by adding vertices for system
 * components and edges representing dependencies between them. Upon successful execution, the graph
 * is sent as output for subsequent nodes to utilize. Error handling occurs if file operations
 * fail, logging relevant messages for troubleshooting.
 */
NodeStatus InitializeBlackboard::tick()
{
    setOutput("executed_strategies", executed_strategies);
    setOutput("adaptations", adaptations);
    return NodeStatus::SUCCESS;
}