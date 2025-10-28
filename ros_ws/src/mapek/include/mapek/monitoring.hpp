#include <mapek/util/graph.hpp>
#include <mapek/util/system_analyzer.hpp>
#include <mapek/json.hpp>
#include <system_interfaces/msg/set_blackboard_group.hpp>
#include <filesystem>
#include "ament_index_cpp/get_package_share_directory.hpp" 
#include <mapek/util/definitions.hpp>

using json = nlohmann::json;

class Monitoring
{
public:
    Monitoring(bool test=false);
    MAPEK_Graph get_initial_graph();
    void update_dependencies_in_graph(MAPEK_Graph &graph);
    void update_key_value_storage(ValueStorePtr random_value_blackboard, const std::shared_ptr<system_interfaces::msg::SetBlackboardGroup> &msg);
private:
    std::unique_ptr<SystemAnalyzer> analyzer;  // Use a smart pointer for delayed initialization
    std::vector<std::pair<std::string, std::string>> blacklisted_nodes;
    std::vector<std::pair<std::string, std::string>> needed_nodes;
    void loadConfig(const std::string& configFilePath);
    std::vector<std::pair<std::string, std::string>> parseNodes(const json& config, const std::string& key);
};