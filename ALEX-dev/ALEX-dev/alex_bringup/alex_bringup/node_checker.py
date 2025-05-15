"""
 _   _                      _ _____                     _            
| \ | | ___ _   _ _ __ __ _| |  ___|__  _   _ _ __   __| |_ __ _   _ 
|  \| |/ _ \ | | | '__/ _` | | |_ / _ \| | | | '_ \ / _` | '__| | | |
| |\  |  __/ |_| | | | (_| | |  _| (_) | |_| | | | | (_| | |  | |_| |
|_| \_|\___|\__,_|_|  \__,_|_|_|  \___/ \__,_|_| |_|\__,_|_|   \__, |
                                                               |___/ 

"""
# ROS Import
import rclpy
from rclpy.node import Node
# ROS Interface Import
from alex_interfaces.msg import ActiveNodes, NodeItem, ActiveNodesStatus
from alex_interfaces.srv import ActivatingNodes
# Python Import
from collections import defaultdict
# Utilities Import
from alex_utilities.common_utilities import change_case

class NodeChecker(Node):
    def __init__(self):
        node_name = change_case(self.__class__.__name__)
        super().__init__(node_name)
        self.logger = self.get_logger()

        self.initilize()
        self.subscribers()
        self.create_service_client()

        self.create_timer(5.0, self.active_nodes)
        self.active_nodes()
    
    def initilize(self):
        self.required_nodes: defaultdict[str, list[NodeItem]] = defaultdict(list)
    
    def subscribers(self,):
        self.active_node_publisher = self.create_publisher(ActiveNodesStatus, "active_node_status", 1)
    
    def create_service_client(self):
        self.create_service(ActivatingNodes, "/activating_nodes", self.handle_activating_nodes)
    
    def handle_activating_nodes(self, request:ActivatingNodes.Request, response:ActivatingNodes.Response):
        self.logger.info(f"Got new activating nodes for pkg: {request.nodes.package_name}")
        self.required_nodes[request.nodes.package_name].extend(request.nodes.nodes)

        response.success = True
        return response

    def active_nodes(self):
        self.logger.debug("Checking for active nodes")
        all_active_nodes = []
        for node_name, name_space in self.get_node_names_and_namespaces():
            spacer = "/" if name_space != "/" else ""
            final_node_name = f"{name_space}{spacer}{node_name}"
            all_active_nodes.append(final_node_name)
        data = '\n'.join(all_active_nodes)
        self.logger.debug(f"Active Nodes: {data}")

        node_status = ActiveNodesStatus()
        for package_name, nodes in self.required_nodes.items():
            active_nodes = ActiveNodes()
            active_nodes.package_name = package_name
            for node in nodes:
                node_item = node
                node_item.is_active = node.node_name in all_active_nodes
                active_nodes.nodes.append(node_item)
            node_status.node_status.append(active_nodes)
        self.active_node_publisher.publish(node_status)
    
def main(args=None):
    rclpy.init(args=args)
    node = NodeChecker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

