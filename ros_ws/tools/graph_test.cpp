#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <stack>
#include <set>
#include <algorithm>


class Node {
public:
    std::string name;
    std::string lifecycle_state;
    std::vector<std::string> rules_defined_by_dsl;
    std::string health_status_heartbeat;
    std::string health_status_rules;
    bool busy;
    std::map<std::string, std::string> arbitrary_data;

    // Default constructor
    Node() : busy(false) {}

    Node(const std::string& name) : name(name), busy(false) {}
};

class Graph {
public:
    std::map<std::string, Node> nodes;
    std::map<std::string, std::vector<std::string>> edges;

    void addNode(const Node& node) {
        nodes[node.name] = node;
    }

    void addEdge(const std::string& from, const std::string& to) {
        edges[from].push_back(to);
    }

    void removeEdge(const std::string& from, const std::string& to) {
        auto& connections = edges[from];
        connections.erase(std::remove(connections.begin(), connections.end(), to), connections.end());
    }

    void updateNode(const std::string& name, const Node& updatedNode) {
        nodes[name] = updatedNode;
    }

    std::set<std::string> getDependencies(const std::string& start) {
        std::set<std::string> visited;
        std::stack<std::string> stack;
        std::set<std::string> dependencies;

        stack.push(start);

        while (!stack.empty()) {
            std::string current = stack.top();
            stack.pop();

            if (visited.find(current) == visited.end()) {
                visited.insert(current);
                for (const auto& neighbor : edges[current]) {
                    stack.push(neighbor);
                    dependencies.insert(neighbor);
                }
            }
        }
        return dependencies;
    }
};

int main() {
    Graph rosGraph;

    Node nodeA("Node A");
    Node nodeB("Node B");
    Node nodeC("Node C");
    Node nodeD("Node D");
    Node nodeE("Node E");

    rosGraph.addNode(nodeA);
    rosGraph.addNode(nodeB);
    rosGraph.addNode(nodeC);
    rosGraph.addNode(nodeD);

    rosGraph.addEdge("Node C", "Node A");
    rosGraph.addEdge("Node C", "Node B");
    rosGraph.addEdge("Node B", "Node A");
    rosGraph.addEdge("Node D", "Node C");

    std::set<std::string> dependencies = rosGraph.getDependencies("Node B");

    std::cout << "Node B has dependencies on: ";
    for (const auto& dep : dependencies) {
        std::cout << dep << " ";
    }
    std::cout << std::endl;

    return 0;
}