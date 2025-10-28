#ifndef INCLUDE_GRAPH_HPP
#define INCLUDE_GRAPH_HPP

#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <stack>
#include <set>
#include <algorithm>
#include <mapek/json.hpp>
#include <mapek/util/definitions.hpp>
#include <fstream>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <system_interfaces/msg/heartbeat.hpp>


class GraphNode {
public:
    const std::string name() const { return _name; }

    uint8_t lifecycle_state() const { return _lifecycle_state; }
    void lifecycle_state(const uint8_t lc_state) { _lifecycle_state = lc_state; }

    uint8_t health_status() const { return _health_status; }
    void health_status(const uint8_t health_status) {
        if (health_status == Heartbeat::HB_STATUS_OK || health_status == Heartbeat::HB_STATUS_DEGRADED || health_status == Heartbeat::HB_STATUS_FAILURE)
        {
            _health_status = health_status;
        }
    }

    bool busy() const { return _busy; }
    void busy(bool busy_level) { _busy = busy_level; }

    // Default constructor
    GraphNode() : _health_status(Heartbeat::HB_STATUS_OK) {}

    GraphNode(const std::string& name) : _name(name), _health_status(Heartbeat::HB_STATUS_OK) {}

private:
    std::string _name;
    uint8_t _lifecycle_state;
    uint8_t _health_status;

    // Here we could think about using the adaptation type in system interfaces to showcase with what 
    // a node is currently busy with. Maybe that helps
    bool _busy{ false };
};

class MAPEK_Graph {
public:
    const std::map<std::string, GraphNode>* nodes() const { return &_nodes; }
    const std::map<std::string, std::set<std::string>>* edges() const { return &_edges; }

    GraphNode& operator[](const std::string& nodeName) {
        try{ 
            return _nodes.at(nodeName);
        }catch(...){
            throw std::runtime_error("no node "+nodeName);
        }
    }

    void addNode(const GraphNode& node) {
        _nodes[node.name()] = node;
    }

    void addEdge(const std::string& from, const std::string& to) {
        try{
            _edges[from].insert(to);
        }catch(...){
            throw std::runtime_error("no node "+from);
        }
    }

    void clearEdges() {
        for (auto& edge: _edges)
        {
            edge.second.clear();
        }
    }

    //void removeEdge(const std::string& from, const std::string& to) {
    //    auto& connections = _edges[from];
    //    connections.erase(std::remove(connections.begin(), connections.end(), to), connections.end());
    //}

    void updateNode(const std::string& name, const GraphNode& updatedNode) {
        try{  
            _nodes[name] = updatedNode;
        }catch(...){
            throw std::runtime_error("no node "+name);
        }
    }

    std::set<std::string> getDependencies(const std::string& start, u_int8_t min_considerable_health_status) const {
        std::set<std::string> visited;
        std::stack<std::string> stack;
        std::set<std::string> dependencies;

        stack.push(start);

        while (!stack.empty()) {
            std::string current = stack.top();
            stack.pop();

            if (visited.find(current) == visited.end()) {
                visited.insert(current);
                if (_edges.find(current) != _edges.end()) {
                    const auto& neighbors = _edges.at(current);

                    for (const auto& neighbor : neighbors) {
                        stack.push(neighbor);
                        if (_nodes.at(neighbor).health_status() >= min_considerable_health_status) {
                            dependencies.insert(neighbor);
                        }
                    }
                }

            }
        }
        return dependencies;
    }

    void printEdges() const {
        for (const auto& [from, targets] : _edges) {
            for (const auto& to : targets) {
                std::cout << from << " -> " << to << std::endl;
            }
        }
    }
    
private:
    std::map<std::string, GraphNode> _nodes;
    std::map<std::string, std::set<std::string>> _edges;
};

#endif