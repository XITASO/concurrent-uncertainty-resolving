import rclpy
from rclpy.node import Node
from python_base_class.node_config import CommunicationTypes
import logging
import networkx as nx
import matplotlib.pyplot as plt
import time
from ros2node.api import get_node_names
from ros2cli.node.strategy import NodeStrategy
import json
import time


class SystemAnalyzer(Node):
    def __init__(self):
        super().__init__('ros2_system_analyzer')

        self.logger = logging.getLogger(__name__)
        # Get all nodes that are present in the ROS system
        self.present_nodes = get_node_names(node=NodeStrategy(None))

        self.get_available_objects = {
            CommunicationTypes.PUBLISHER: self.get_publisher_names_and_types_by_node,
            CommunicationTypes.SUBSCRIPTION: self.get_subscriber_names_and_types_by_node,
            CommunicationTypes.SERVICE: self.get_service_names_and_types_by_node,
            CommunicationTypes.SERVICE_CLIENT: self.get_client_names_and_types_by_node,
        }

        self.communication_dict = {
            CommunicationTypes.PUBLISHER: {},
            CommunicationTypes.SUBSCRIPTION: {},
            CommunicationTypes.SERVICE: {},
            CommunicationTypes.SERVICE_CLIENT: {},
        }

        self.graph = nx.DiGraph()

    def analyze_system(self):
        # Collect all communication channels in a dictionary
        nodes = list()
        for node in self.present_nodes:
            node_name = node.name
            node_namespace = node.namespace
            if "blackboard_setter" in node_name or self.get_name() in node_name:
                continue
            
            time.sleep(1)
            #do_this_node = input(f"Do you want to add the node {node_namespace}/{node_name} in your dependency graph? [y/n]")
            #if not do_this_node == "y":
            #    continue
            nodes.append(self.generate_name(node_name, node_namespace))
            print(f"Analyzing node: {node_name} in namespace: {node_namespace}")
            for comm_type in CommunicationTypes:
                if comm_type == CommunicationTypes.ACTION_CLIENT or comm_type == CommunicationTypes.ACTION_SERVER:
                    continue
                try:
                    available_comm_channels = self.get_available_objects[comm_type](node_name, node_namespace)
                    self.collect_communication_channels(node_name, node_namespace, comm_type, available_comm_channels)
                except:
                    print(f'Failed {node_name}')
                    exit()

        # Analyze connections and build the graph
        self.build_graph(nodes)

    def collect_communication_channels(self, node_name, node_namespace, comm_type, comm_channels):
        for channel_name, channel_type in comm_channels:
            if channel_name not in self.communication_dict[comm_type]:
                self.communication_dict[comm_type][channel_name] = []
            self.communication_dict[comm_type][channel_name].append((node_name, node_namespace, channel_type))

    def generate_name(self, name: str, namespace: str):
        if namespace == "/":
            return name
        else:
            return f"{namespace}/{name}"

    def build_graph(self, nodes: list):
        for node in nodes:
            self.graph.add_node(node)
        
        # Build the graph based on collected communication channels
        for channel_name in self.communication_dict[CommunicationTypes.PUBLISHER]:
            publishers = self.communication_dict[CommunicationTypes.PUBLISHER][channel_name]
            subscribers = self.communication_dict[CommunicationTypes.SUBSCRIPTION].get(channel_name, [])
            for pub_node, pub_namespace, pub_type in publishers:
                pub_node_name = self.generate_name(pub_node, pub_namespace)
                for sub_node, sub_namespace, sub_type in subscribers:
                    sub_node_name = self.generate_name(sub_node, sub_namespace)
                    self.graph.add_edge(sub_node_name, pub_node_name, label=pub_type)

        for channel_name in self.communication_dict[CommunicationTypes.SERVICE]:
            services = self.communication_dict[CommunicationTypes.SERVICE][channel_name]
            clients = self.communication_dict[CommunicationTypes.SERVICE_CLIENT].get(channel_name, [])
            for service_node, service_namespace, service_type in services:
                service_name = self.generate_name(service_node,service_namespace)
                for client_node, client_namespace, client_type in clients:
                    client_name = self.generate_name(client_node, client_namespace)
                    self.graph.add_edge(client_name, service_name, label=service_type)

    def display_graph(self):
        pos = nx.spring_layout(self.graph)
        nx.draw(self.graph, pos, with_labels=True, node_size=2000, node_color="lightblue", font_size=10, font_weight="bold")
        with open("managed_subsystem.json", "w") as f:
            json.dump(nx.json_graph.node_link_data(self.graph, edges="links"), f, indent=2)
            
        plt.title("ROS2 System Communication Graph")
        plt.savefig("Test.pdf")

if __name__ == "__main__":
    rclpy.init()
    time.sleep(1)
    analyzer = SystemAnalyzer()
    analyzer.analyze_system()
    analyzer.display_graph()
