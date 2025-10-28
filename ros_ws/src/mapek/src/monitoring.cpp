#include <mapek/monitoring.hpp>
#include <filesystem>

Monitoring::Monitoring(bool test)
{
  std::string package_name = "mapek";

  std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
  std::string config_file_relative_path = "config/graph_config.json";

  // Form the full file path
  std::filesystem::path config_file_path = std::filesystem::path(package_share_directory) / config_file_relative_path;

  // Convert the full path to a string

  std::string config_file_path_str = config_file_path.string();
  if (!test)
  {
    if (std::filesystem::exists(config_file_path_str))
    {
      loadConfig(config_file_path_str);
      analyzer = std::make_unique<SystemAnalyzer>(needed_nodes, blacklisted_nodes);
    }
  }
}

MAPEK_Graph Monitoring::get_initial_graph()
{
  return analyzer->get_graph();
}

void Monitoring::update_dependencies_in_graph(MAPEK_Graph &graph)
{
  analyzer->update_graph(graph);
}

void Monitoring::update_key_value_storage(ValueStorePtr random_value_blackboard, const std::shared_ptr<system_interfaces::msg::SetBlackboardGroup> &msg)
{
  for (const rcl_interfaces::msg::Parameter &param_msg : msg->bb_params)
  {
    const std::string &key = param_msg.name;
    const ParamValue &value = param_msg.value;

    auto it = random_value_blackboard->find(key);
    if (it != random_value_blackboard->end())
    {
      // If the key exists, update the value
      it->second = value;
    }
    else
    {
      // If the key does not exist, add it to the map
      random_value_blackboard->insert({key, value});
    }
  }
}

std::vector<std::pair<std::string, std::string>> Monitoring::parseNodes(const json &config, const std::string &key)
{
  std::vector<std::pair<std::string, std::string>> nodes;

  if (config.contains(key))
  {
    const auto &jsonNodes = config[key];
    for (const auto &node : jsonNodes)
    {
      std::string nodeName = node["node_name"];
      std::string namespaceName = node["namespace"];
      nodes.push_back(std::make_pair(nodeName, namespaceName));
    }
  }

  return nodes;
}

void Monitoring::loadConfig(const std::string &configFilePath)
{
  std::ifstream configFile(configFilePath);
  json config;
  configFile >> config;

  blacklisted_nodes = parseNodes(config, "blacklisted_nodes");
  needed_nodes = parseNodes(config, "needed_nodes");

  // Output the nodes to verify the result
  std::cout << "Blacklisted Nodes:\n";
  for (const auto &node : blacklisted_nodes)
  {
    std::cout << node.first << " " << node.second << std::endl;
  }

  std::cout << "\nNeeded Nodes:\n";
  for (const auto &node : needed_nodes)
  {
    std::cout << node.first << " " << node.second << std::endl;
  }
}