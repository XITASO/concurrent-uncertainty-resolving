#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <unordered_map>
#include <map>
#include <mapek/util/graph.hpp>

class SystemAnalyzer : public rclcpp::Node
{
public:
    SystemAnalyzer(std::vector<std::pair<std::string, std::string>>& needed_nodes, std::vector<std::pair<std::string, std::string>> & blacklisted_nodes);
    MAPEK_Graph get_graph();   
    void update_graph(MAPEK_Graph &graph);

private:
    std::vector<std::pair<std::string, std::string>> present_nodes;
    std::vector<std::pair<std::string, std::string>> blacklisted_nodes;
    std::vector<std::pair<std::string, std::string>> needed_nodes;
    std::unordered_map<std::string, std::vector<std::string>> publishers;
    std::unordered_map<std::string, std::vector<std::string>> subscribers;
    std::unordered_map<std::string, std::vector<std::string>> services;
    std::unordered_map<std::string, std::vector<std::string>> clients;

    void initialize(std::vector<std::pair<std::string, std::string>> needed_nodes);
    void analyze_system(std::vector<std::pair<std::string, std::string>> blacklisted_nodes);
    std::vector<std::string> get_publishers_for_node(const std::string& node_name, const std::string& node_namespace);
    std::vector<std::string> get_subscribers_for_node(const std::string& node_name, const std::string& node_namespace);
    std::vector<std::string> get_services_for_node(const std::string& node_name, const std::string& node_namespace);
    std::vector<std::string> get_clients_for_node(const std::string& node_name, const std::string& node_namespace);
    MAPEK_Graph build_graph();
    void add_edges_to_graph(MAPEK_Graph &graph);
    std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph;
};