#include <gtest/gtest.h>
#include <stdexcept>
#include <string>
#include <set>
#include <map>
#include <mapek/util/graph.hpp>
#include "test_utils.hpp"

// Test for adding nodes and verifying their existence in the graph
TEST(test_mapek_graph, test_add_nodes) {
    // Arrange
    MAPEK_Graph systemGraph;

    // Act
    systemGraph.addNode(GraphNode("A"));
    systemGraph.addNode(GraphNode("B"));
    systemGraph.addNode(GraphNode("C"));

    // Assert
    const auto* nodes = systemGraph.nodes();
    ASSERT_TRUE(nodes->find("A") != nodes->end());
    ASSERT_TRUE(nodes->find("B") != nodes->end());
    ASSERT_TRUE(nodes->find("C") != nodes->end());
}

// Test for adding edges and verifying their existence in the graph
TEST(test_mapek_graph, test_add_edges) {
    // Arrange
    MAPEK_Graph systemGraph = get_simple_graph();

    // Act
    systemGraph.addEdge("A", "B");
    systemGraph.addEdge("B", "C");

    // Assert
    const auto* edges = systemGraph.edges();
    ASSERT_TRUE(edges->find("A") != edges->end());
    ASSERT_TRUE(edges->at("A").find("B") != edges->at("A").end());
    ASSERT_TRUE(edges->find("B") != edges->end());
    ASSERT_TRUE(edges->at("B").find("C") != edges->at("B").end());
}

// Test for clearing edges
TEST(test_mapek_graph, test_clear_edges) {
    // Arrange
    MAPEK_Graph systemGraph = get_simple_graph();
    systemGraph.addEdge("A", "B");
    systemGraph.addEdge("B", "C");

    // Act
    systemGraph.clearEdges();

    // Assert
    const auto* edges = systemGraph.edges();
    for (const auto& edge : *edges) {
        ASSERT_TRUE(edge.second.empty());
    }
}

// Test for getting dependencies based on health status
TEST(test_mapek_graph, test_get_dependencies) {
    // Arrange
    MAPEK_Graph systemGraph = get_simple_graph();

    const std::string nodeName = "A";
    // Set health statuses
    systemGraph[nodeName].health_status(Heartbeat::HB_STATUS_OK);
    systemGraph["B"].health_status(Heartbeat::HB_STATUS_OK);
    systemGraph["C"].health_status(Heartbeat::HB_STATUS_OK);

    // Act
    std::set<std::string> dependencies = systemGraph.getDependencies(nodeName, Heartbeat::HB_STATUS_OK);

    // Assert
    ASSERT_TRUE(dependencies.find("B") != dependencies.end());
    ASSERT_TRUE(dependencies.find("C") != dependencies.end());
}

// Test for exception when accessing a non-existent node
TEST(test_mapek_graph, test_access_non_existent_node) {
    // Arrange
    MAPEK_Graph systemGraph = get_simple_graph();

    // Act and Assert
    EXPECT_THROW(systemGraph["NonExistentNode"], std::runtime_error);
}