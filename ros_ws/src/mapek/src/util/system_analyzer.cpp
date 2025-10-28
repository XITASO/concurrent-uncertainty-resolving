#include <mapek/util/system_analyzer.hpp>

SystemAnalyzer::SystemAnalyzer(std::vector<std::pair<std::string, std::string>> &needed_nodes, std::vector<std::pair<std::string, std::string>> &blacklisted_nodes) : Node("ros2_system_analyzer")
{
    node_graph = this->get_node_graph_interface();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    blacklisted_nodes = blacklisted_nodes;
    needed_nodes = needed_nodes;
    initialize(needed_nodes);
    analyze_system(blacklisted_nodes);
}

void SystemAnalyzer::initialize(std::vector<std::pair<std::string, std::string>> needed_nodes)
{
    // TODO remove the blacklisted nodes
    while (true)
    {
        present_nodes = node_graph->get_node_names_and_namespaces();
        bool any_node_missing = false;
        for (const auto &node : needed_nodes)
        {
            if (std::find(present_nodes.begin(), present_nodes.end(), node) == present_nodes.end())
            {
                any_node_missing = true;
                break;
            }
        }
        if (any_node_missing == false)
            break;
    }
}

void SystemAnalyzer::analyze_system(std::vector<std::pair<std::string, std::string>> blacklisted_nodes)
{
    // RCLCPP_DEBUG(this->get_logger(), "Starting system analysis...");
    for (const auto &present_node : present_nodes)
    {
        if (std::find(blacklisted_nodes.begin(), blacklisted_nodes.end(), present_node) != blacklisted_nodes.end())
        {
            RCLCPP_DEBUG(this->get_logger(), "Skipping %s", present_node.first.c_str());
            continue;
        }
        std::string node_name = present_node.first;
        std::string node_namespace = present_node.second;

        // RCLCPP_DEBUG(this->get_logger(), "Analyzing node: %s", node_name.c_str());

        // Retrieve publishers for the node using node_graph functions.
        try
        {
            publishers[node_namespace + "/" + node_name] = get_publishers_for_node(node_name, node_namespace);

            // Retrieve subscribers for the node.
            subscribers[node_namespace + "/" + node_name] = get_subscribers_for_node(node_name, node_namespace);

            // Retrieve services for the node.
            services[node_namespace + "/" + node_name] = get_services_for_node(node_name, node_namespace);

            // Retrieve clients for the node.
            clients[node_namespace + "/" + node_name] = get_clients_for_node(node_name, node_namespace);
            // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        catch(...)
        {
            continue;
        }
    }
    build_graph();
}

std::vector<std::string> SystemAnalyzer::get_publishers_for_node(const std::string &node_name, const std::string &node_namespace)
{
    std::vector<std::string> pubs;

    auto publisher_map = node_graph->get_publisher_names_and_types_by_node(node_name, node_namespace, false);
    for (const auto &pair : publisher_map)
    {
        for (const auto &type : pair.second)
        {
            pubs.push_back(node_namespace + pair.first + ":" + type);
        }
    }

    return pubs;
}

std::vector<std::string> SystemAnalyzer::get_subscribers_for_node(const std::string &node_name, const std::string &node_namespace)
{
    std::vector<std::string> subs;

    auto subscriber_map = node_graph->get_subscriber_names_and_types_by_node(node_name, node_namespace, false);
    for (const auto &pair : subscriber_map)
    {
        for (const auto &type : pair.second)
        {
            subs.push_back(node_namespace + pair.first + ":" + type);
        }
    }

    return subs;
}

std::vector<std::string> SystemAnalyzer::get_services_for_node(const std::string &node_name, const std::string &node_namespace)
{
    std::vector<std::string> svcs;

    auto service_map = node_graph->get_service_names_and_types_by_node(node_name, node_namespace);
    for (const auto &pair : service_map)
    {
        for (const auto &type : pair.second)
        {
            svcs.push_back(pair.first + ":" + type);
        }
    }

    return svcs;
}

std::vector<std::string> SystemAnalyzer::get_clients_for_node(const std::string &node_name, const std::string &node_namespace)
{
    std::vector<std::string> clnts;

    auto client_map = node_graph->get_client_names_and_types_by_node(node_name, node_namespace);
    for (const auto &pair : client_map)
    {
        for (const auto &type : pair.second)
        {
            clnts.push_back(pair.first + ":" + type);
        }
    }

    return clnts;
}

void SystemAnalyzer::add_edges_to_graph(MAPEK_Graph &graph)
{
    // Add edges between publishers and subscribers
    for (const auto &pub_node : publishers)
    {
        const auto &publisher_node_name = pub_node.first;
        for (const auto &pub_topic_type : pub_node.second)
        {
            const std::string &pub_topic = pub_topic_type.substr(0, pub_topic_type.find(":"));

            for (const auto &sub_node : subscribers)
            {
                const auto &subscriber_node_name = sub_node.first;
                for (const auto &sub_topic_type : sub_node.second)
                {
                    const std::string &sub_topic = sub_topic_type.substr(0, sub_topic_type.find(":"));

                    // If publisher topic matches subscriber topic, add an edge
                    if (pub_topic == sub_topic)
                    {
                        graph.addEdge(subscriber_node_name, publisher_node_name);
                        RCLCPP_DEBUG(this->get_logger(), "Create connection between %s and %s", publisher_node_name.c_str(), subscriber_node_name.c_str());
                    }
                }
            }
        }
    }

    // Add edges between clients and services
    for (const auto &client_node : clients)
    {
        const auto &client_node_name = client_node.first;
        for (const auto &client_service_type : client_node.second)
        {
            const std::string &client_service = client_service_type.substr(0, client_service_type.find(":"));

            for (const auto &service_node : services)
            {
                const auto &service_node_name = service_node.first;
                for (const auto &service_service_type : service_node.second)
                {
                    const std::string &service_service = service_service_type.substr(0, service_service_type.find(":"));

                    // If client service matches service topic, add an edge
                    if (client_service == service_service)
                    {
                        graph.addEdge(service_node_name, client_node_name);
                        RCLCPP_DEBUG(this->get_logger(), "Create connection between %s and %s", client_node_name.c_str(), service_node_name.c_str());
                    }
                }
            }
        }
    }
}

MAPEK_Graph SystemAnalyzer::build_graph()
{
    // Implement logic for building the communication graph
    RCLCPP_DEBUG(this->get_logger(), "Building the communication graph...");
    MAPEK_Graph graph;

    // Create graph nodes for each present node
    for (const auto &present_node : present_nodes)
    {
        const std::string &node_name = present_node.first;
        const std::string &node_namespace = present_node.second;
        graph.addNode(GraphNode(node_namespace + "/" + node_name));
        RCLCPP_DEBUG(this->get_logger(), "Creating node for %s", node_name.c_str());
    }

    add_edges_to_graph(graph);

    RCLCPP_DEBUG(this->get_logger(), "Graph has been successfully built with %lu edges.", graph.edges()->size());
    return graph;
}

MAPEK_Graph SystemAnalyzer::get_graph()
{
    analyze_system(blacklisted_nodes);

    return build_graph();
}

void SystemAnalyzer::update_graph(MAPEK_Graph &graph)
{
    analyze_system(blacklisted_nodes);
    graph.clearEdges();
    add_edges_to_graph(graph);
}
