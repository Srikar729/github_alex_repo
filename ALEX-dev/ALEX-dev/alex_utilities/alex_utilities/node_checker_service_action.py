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
from rclpy.client import Client
from rclpy.node import Node as ROSNode
from rclpy.executors import SingleThreadedExecutor, Executor
# ROS Interface Import
from alex_interfaces.msg import NodeItem
from alex_interfaces.srv import ActivatingNodes
# Python Import
import asyncio
from launch_ros.actions import Node
from launch.utilities import create_future
from launch import Action, LaunchContext, LaunchDescription

class ActivatingNodeAction(Action):
    """
    Custom ROS 2 Launch Action to check nodes liveliness via a service call.

    This action identifies all nodes within a given LaunchDescription and sends their
    names to a ROS 2 service for checkes.

    see docs: alex_utilities/docs/node_checker_service_action.md
    """

    def __init__(self, ld: LaunchDescription, package_name: str):
        """
        Initialize the ActivatingNodeAction.

        :param ld: The LaunchDescription containing the nodes to be activated.
        :param package_name: The name of the package associated with the nodes.
        """
        super().__init__()
        self.service_name = "/activating_nodes"
        self.ld = ld
        self.package_name = package_name
        self._completed_future = None
        self._canceled = False

    def _get_node_names(self, context: LaunchContext, ld: LaunchDescription):
        """
        Extract node names from the LaunchDescription.

        :param context: The LaunchContext for substitutions.
        :param ld: The LaunchDescription containing the nodes.
        :return: A set of node names.
        """
        all_nodes = set()
        for entity in ld.entities:
            if isinstance(entity, Node):
                # Resolve namespace and name substitutions
                entity._perform_substitutions(context)
                node_name = entity.node_name.removeprefix("<node_namespace_unspecified>")
                all_nodes.add(node_name)
        return all_nodes

    def _ensure_shared_rclpy_context(self, context: LaunchContext):
        """
        Ensure a shared RCL context exists in the LaunchContext.

        :param context: The LaunchContext to access shared state.
        :return: The shared RCL context.
        """
        if not hasattr(context, '_shared_rclpy_context'):
            context._shared_rclpy_context = rclpy.Context()
            rclpy.init(context=context._shared_rclpy_context)
        return context._shared_rclpy_context

    def _create_shared_node(self, shared_context: rclpy.Context):
        """
        Create a shared ROS node using the provided context.

        :param shared_context: The RCL context for the node.
        :return: The created ROS node.
        """
        return rclpy.create_node(f'activating_node_action_{self.package_name}', context=shared_context)

    def _create_service_client(self, node: ROSNode):
        """
        Create a client for the ActivatingNodes service.

        :param node: The ROS node to create the client on.
        :return: The service client.
        """
        return node.create_client(ActivatingNodes, self.service_name)

    async def _wait_for_service(self, node: ROSNode, client: Client):
        """
        Wait for the service to become available.

        :param node: The ROS node to log messages.
        :param client: The service client to check availability.
        """
        while not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('Waiting for the activating_nodes service...')
            await asyncio.sleep(1)

    def _prepare_request(self, context: LaunchContext):
        """
        Prepare the service request by collecting node names from the LaunchDescription.

        :param context: The LaunchContext to resolve substitutions.
        :return: The prepared service request.
        """
        request = ActivatingNodes.Request()
        request.nodes.package_name = self.package_name

        # Collect all node names from the LaunchDescription
        all_nodes = self._get_node_names(context, self.ld)
        for item in all_nodes:
            node_item = NodeItem()
            node_item.node_name = item
            node_item.required = True
            request.nodes.nodes.append(node_item)

        return request

    async def _send_service_request(self, node: ROSNode, executor: Executor, client: Client, request: ActivatingNodes.Request):
        """
        Send the prepared service request asynchronously.

        :param node: The ROS node.
        :param client: The service client to call.
        :param request: The prepared service request.
        """
        future = client.call_async(request)

        while not future.done():
            executor.spin_once(timeout_sec=0.1)
            await asyncio.sleep(0.5)  # Yield control to the asyncio event loop

        # Process the result of the future
        response = future.result()
        node.get_logger().info(f"Service response received: {response}")

    def _mark_action_complete(self):
        """Mark the action as complete by resolving the associated future."""
        if self._completed_future and not self._completed_future.done():
            self._completed_future.set_result(None)

    def cancel(self):
        """Cancel the action and mark it as completed."""
        self._canceled = True
        if self._completed_future and not self._completed_future.done():
            self._completed_future.set_result(None)

    async def _call_service(self, context: LaunchContext):
        """
        Perform the asynchronous service call to activate nodes.

        :param context: The LaunchContext to access shared state and the asyncio loop.
        """
        shared_context = self._ensure_shared_rclpy_context(context)
        node = self._create_shared_node(shared_context)
        executor = SingleThreadedExecutor(context=shared_context)
        executor.add_node(node)

        client = self._create_service_client(node)
        await self._wait_for_service(node, client)
        request = self._prepare_request(context)
        await self._send_service_request(node, executor, client, request)

        # Mark the action as complete
        self._mark_action_complete()
        node.get_logger().info("Process done, Destroying node")
        node.destroy_node()

    def execute(self, context: LaunchContext):
        """
        Execute the action and register the service call coroutine in the asyncio loop.

        :param context: The LaunchContext for the action.
        """
        # Initialize a future to track completion
        self._completed_future = create_future(context.asyncio_loop)

        # Schedule the service call in the asyncio loop
        context.asyncio_loop.create_task(self._call_service(context))

        return None
