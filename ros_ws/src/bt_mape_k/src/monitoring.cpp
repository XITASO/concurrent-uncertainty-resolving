#include <bt_mape_k/monitoring.hpp>
using namespace BT;

/**
 * @brief Implements the behavior tree node's tick action for monitoring and updating the blackboard based on
 *        incoming messages.
 * 
 * @param msg A shared pointer to a SetBlackboardGroup message containing parameters to update on the blackboard.
 * 
 * @return NodeStatus indicating the result of the action execution, returns NodeStatus::SUCCESS if the operation
 *         completes successfully, or NodeStatus::FAILURE if there's an issue with accessing the blackboard.
 * 
 * The onTick function retrieves the current time and sets it as an output on the "clock" port of the node.
 * If a message is provided, it iterates over the message's parameters, logs the parameter details for debugging
 * purposes, and sets these values on the blackboard based on their data types (bool, integer, double, or string).
 * The function checks if a blackboard object is available; otherwise, it logs an error and returns a failure status.
 */
NodeStatus BTMonitoring::onTick(const std::shared_ptr<system_interfaces::msg::SetBlackboardGroup> &msg)
{
  // Access the current time
  rclcpp::Time current_time = node_handle->now();

  // Convert the time to milliseconds
  long long current_time_ms = current_time.nanoseconds();
  setOutput("clock", current_time_ms);
  MAPEK_Graph graph = getInput<MAPEK_Graph>("system_state_graph").value();
  mapek_monitoring->update_dependencies_in_graph(graph);
  setOutput("sytem_state_graph", graph);

  if (!msg) // empty if no new message received, since the last tick
  {
    return BT::NodeStatus::SUCCESS;
  }

  ValueStorePtr random_value_blackboard = getInput<ValueStorePtr>("value_store").value();
  mapek_monitoring->update_key_value_storage(random_value_blackboard, msg);
  setOutput("value_store", random_value_blackboard);

  return BT::NodeStatus::SUCCESS;
}