
# ActivatingNodeAction for ROS 2

The `ActivatingNodeAction` is a custom **ROS 2 Launch Action** that extracts nodes from a `LaunchDescription` and sends them to the `/activating_nodes` ROS 2 service for further processing. It uses Python’s `asyncio` to perform this operation asynchronously, ensuring the action does not block the main ROS 2 launch process.

---

## Features

1. **Node Extraction**:
   - Automatically identifies all `Node` entities in a `LaunchDescription`.
   - Resolves node names and namespaces dynamically.

2. **Asynchronous Execution**:
   - Uses `asyncio` to interact with the ROS 2 service without holding up the main thread of the launch process.
   - Integrates seamlessly with the ROS 2 launch system.

3. **ROS 2 Service Communication**:
   - Sends a service request to `/activating_nodes`, containing the list of extracted nodes and their package name.
   - Handles the response asynchronously.

4. **Non-blocking Resource Management**:
   - Ensures that temporary ROS 2 resources (like nodes and contexts) are cleaned up after the operation.

---

## How It Works

### Workflow

1. **Node Discovery**:
   - Parses the `LaunchDescription` to extract all nodes defined within it.
   - Resolves node names and namespaces to prepare a list of nodes.

2. **Service Communication**:
   - Creates a temporary ROS 2 node to communicate with the `/activating_nodes` service.
   - Prepares a service request containing:
     - The package name.
     - A list of node names and their attributes.

3. **Asynchronous Operation**:
   - Waits for the service to be available.
   - Sends the request and waits for the response without blocking the main launch process.

4. **Cleanup**:
   - Destroys the temporary ROS 2 node after the service call is completed or canceled.

---

## Usage

### 1. Add the Action to a Launch Script

Here’s an example of how to use `ActivatingNodeAction` in a launch file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from activating_node_action import ActivatingNodeAction

def generate_launch_description():
    # Define the nodes to be launched
    node1 = Node(
        package="example_package",
        executable="example_node",
        name="example_node_1"
    )

    node2 = Node(
        package="example_package",
        executable="example_node",
        name="example_node_2"
    )

    # Create a LaunchDescription
    ld = LaunchDescription([
        node1,
        node2
    ])

    # Add the ActivatingNodeAction
    activate_action = ActivatingNodeAction(ld=ld, package_name="example_package")

    # Return the LaunchDescription including the action
    return LaunchDescription([
        ld,
        activate_action
    ])
```
---

## Code Breakdown

### Constructor
- Initializes the `ActivatingNodeAction` with a `LaunchDescription` and package name.
- Sets up the service name as `/activating_nodes`.

### Execution
- The `execute()` method registers the `_call_service` coroutine with the asyncio loop to perform the service call in the background.

### Async Service Call
1. **Node Discovery**:
   - `_get_node_names`: Extracts node names from the `LaunchDescription` and resolves their namespaces.
2. **ROS Setup**:
   - `_create_shared_node`: Creates a temporary ROS 2 node.
   - `_create_service_client`: Creates a service client for `/activating_nodes`.
3. **Service Request**:
   - `_prepare_request`: Prepares the service request with the node names and package name.
   - `_send_service_request`: Sends the service request and logs the response.

### Asynchronous Workflow
- Uses `asyncio` to wait for service availability and handle the service response without blocking the launch thread.

---
