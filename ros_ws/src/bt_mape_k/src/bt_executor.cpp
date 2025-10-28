#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "bt_mape_k/analysing.hpp"
#include "bt_mape_k/planning.hpp"
#include "bt_mape_k/register_adaptation.hpp"
#include "bt_mape_k/monitoring.hpp"
#include "bt_mape_k/execution.hpp"
#include "bt_mape_k/initializing.hpp"
#include "bt_mape_k/script_condition_memory.hpp"

#include <thread>

using namespace BT;

int main(int argc, char* argv[]){
  std::string file_folder = ament_index_cpp::get_package_share_directory("bt_mape_k") + "/bts/";
  std::cout << file_folder << std::endl;

  std::string rule_set = "test_rules.txt";
  //if (argc>=2){
  //  rule_set = argv[1];
  //}

  std::string rules_path = file_folder+rule_set;
  std::string bt_path = file_folder+"main_bt.xml";

  std::cout<<rules_path<<std::endl;

  rclcpp::init(argc, argv);
  auto nh_monitoring = std::make_shared<rclcpp::Node>("bt_monitoring_subscriber");
  BehaviorTreeFactory factory;

  // Monitoring
  RosNodeParams monitoring_ros_node_params;
  monitoring_ros_node_params.nh = nh_monitoring;
  monitoring_ros_node_params.default_port_value = "blackboard_group";
  factory.registerNodeType<BTMonitoring>("Monitoring", monitoring_ros_node_params);
  factory.registerNodeType<InitializeBlackboard>("InitializeBlackboard");

  // Analysing and Planning
  factory.registerNodeType<ScriptConditionWithMemory>("ScriptConditionWithMemory");
  factory.registerNodeType<AnalysisDecorator>("AnalysisDecorator", rules_path);
  factory.registerNodeType<RegisterAdaptation>("RegisterAdaptation");
  factory.registerNodeType<PlanningDecorator>("PlanningDecorator");

  factory.registerNodeType<BTExecuteCommChange>("ExecuteCommunicationAdaptation");
  factory.registerNodeType<BTExecuteLifeCycle>("ExecuteLifecycle");
  factory.registerNodeType<BTExecuteParametrization>("ExecuteParametrization");
  factory.registerNodeType<BTExecuteRedeploy>("ExecuteRedeploy");
  factory.registerNodeType<BTExecuteModeChange>("ExecuteModeChange");

  factory.registerBehaviorTreeFromFile(bt_path); 

  auto main_tree = factory.createTree("Main");

  std::chrono::milliseconds intervall_time = std::chrono::milliseconds(200);

  while(rclcpp::ok())
  {
    auto tick_start_time = std::chrono::steady_clock::now();
    main_tree.tickWhileRunning(intervall_time);
    
    // Calculate sleep time
    auto tick_end_time = std::chrono::steady_clock::now();
    auto tick_duration = std::chrono::duration_cast<std::chrono::milliseconds>(tick_end_time - tick_start_time);
    std::chrono::milliseconds sleep_time = intervall_time - tick_duration;

    // Only sleep if the tick duration was less than 100ms
    if (sleep_time > std::chrono::milliseconds(0)) {
      std::this_thread::sleep_for(sleep_time);
    }
    else {
      std::cout << "[Warning] BT frequency is too high, execution took longer than expected." << std::endl;
    }
  }

  return 0;
}