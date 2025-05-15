# ALEX Externals

The `alex_externals` package in the ALEX system handles communication and interfacing with various external devices, allowing for seamless integration with hardware such as cameras, robotic arms, actuators, and peripheral sensors.

This package provides multiple nodes, each dedicated to controlling or receiving data from a specific external device, ensuring modularity and adaptability for adding or modifying devices.

## Nodes Documentation

Each node has its unique functionality and configuration parameters.

For some serial integrations using Arduino, we have implemented prefixes in the serial input from devices to identify the data source. The prefixes for each device are configured in the file [external_constants.py](/alex_utilities/alex_utilities/external_constants.py).

### 1. Serial Interface Lifecycle

This node manages the serial interface lifecycle, allowing it to connect, monitor, and disconnect serial devices autonomously.

Refer to the detailed [Serial Interface Documentation](/alex_externals/docs/Serial%20Interface.md) for lifecycle commands and usage.

### 2. Doossan Arm Control

This ROS2 Python node allows you to control a Doosan robotic arm. It provides functionalities such as moving the arm, controlling the gripper, and performing specific tasks like stirring and packing/unpacking. The node interfaces with Doosan services to execute motion commands and handle the gripper.

> **NOTE**: Make sure to run dsr launch file `ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.1.140 model:=a0509 color:=blue` 

#### Key Features:
- **Move Arm:** Moves the robot to a specific position in space.
- **Gripper Control:**
  - **Hold:** Holds the gripper with a specified group (`picking_group`).
  - **Release:** Releases the gripper.
  - **Turn:** Rotates the gripper to a given angle.
- **Relative Movement:** Moves the robot by a small offset relative to its current position.
- **Stir In-place:** Executes a stirring motion by alternating gripper rotations.
- **Pack/Unpack:** Moves the arm to predefined positions for packing or unpacking.
- **Customizable Parameters:** Adjusts max connection attempts, arm velocity, acceleration, and gripper settings.

#### Parameters:
The following ROS2 parameters are available for customization:

- **client_max_attempt_count**: Maximum attempts to connect to Doosan client services (default: 5).
- **gripper_type**: Type of gripper attached (e.g., AG-95).
- **gripper_delay_time**: Time to wait after gripper actions (default: 2 seconds).
- **doosan_arm_delay**: Delay after arm movements to avoid crashes (default: 0.5 seconds).
- **default_gripper_hold_angle**: Default angle for holding the gripper (default: -90 degrees).
- **default_velocity**: Default velocity for arm movements (default: 100.5 degrees/sec).
- **default_acceleration**: Default acceleration for arm movements (default: 60.0 degrees/sec²).
- **max_velocity**: Maximum velocity for arm movements (default: 100.0 degrees/sec).
- **max_acceleration**: Maximum acceleration for arm movements (default: 60.0 degrees/sec²).
- **robot_pack_position**: Joint positions for packing the robot (default: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).
- **robot_unpack_position**: Joint positions for unpacking the robot (default: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).


#### Service Client Connections:
The node automatically connects to the following Doosan services:
- **move_joint**: Moves the arm to a specified joint position.
- **move_jointx**: Moves the arm using Cartesian coordinates.
- **set_tool_digital_output**: Controls the gripper (e.g., hold or release).
- **get_current_posj**: Retrieves the current joint position of the arm.
- **get_current_posx**: Retrieves the current Cartesian position of the arm.

#### Services:
The node provides the following services via ROS2 for controlling the robot:

#### `doosan_arm_control` (ArmCommand)
- **Command Options:**
  - **MOVE**: Move the arm to a specific position (x, y, z).
  - **PICK**: Activate the gripper to pick an item using a specific `picking_group`. [Refer document](/alex_externals/docs/AG-95%20Gripper%20setup.md) to change group.
  - **DROP**: Release the item using the gripper.
  - **TURN**: Rotate the gripper to a specified angle.
  - **RELATIVE_MOVEMENT**: Move the arm by a specified offset (x, y, z) relative to its current position.
  - **STIR**: Perform a stirring motion with the gripper.
  - **PACK**: Move the arm to the packing position.
  - **UNPACK**: Move the arm to the unpacking position.

#### Example usage

Moving the cobot:
```bash
ros2 service call /doosan_arm_control doosan_arm_control_msgs/srv/ArmCommand "{
    command: move,
    position: {x: 0.5, y: 0.3, z: 0.2}
}"
```

Closing the gripper:
```bash
ros2 service call /doosan_arm_control doosan_arm_control_msgs/srv/ArmCommand "{
    command: pick,
    picking_group: 4,
}"
```


### 3. alex liquid doser

The `alex liquid doser` node responsible for managing and executing precise liquid dosing commands to an external dispensing device. This node communicates with the device over a serial connection, controlling liquid volumes for tasks requiring accurate measurement and dispensing.

#### Key Features

- **Configurable Parameters**: The node allows several parameters to be set through ROS 2 parameters, including:
  - `serial_node_name`: Name of the serial node for communication.
  - `max_acknowledgement_duration`: Maximum time to wait for an acknowledgment.
  - `max_completion_duration`: Maximum time to wait for the task completion.

- **Publishers and Subscribers**:
  - **Publishers**:
    - `to_serial`: Publishes `SerialMessage` to send commands to the serial device.
    - `liquid_dosing_status`: Publishes `LiquidDosingStatus` updates on the dosing process.
  - **Subscribers**:
    - `from_serial`: Listens to `SerialMessage` from the serial device and processes incoming responses.

- **Services**:
  - **start_liquid_dosing**: Service (`LiquidDoserCommand`) to initiate a dosing command, which supports a range of commands to start, stop, or check dosing status, among other options.

#### Lifecycle Management

The `LiquidDoser` node utilizes ROS 2 lifecycle management to configure and activate the dosing device as follows:
1. **Configure**: Sets up the device's initial state.
2. **Activate**: Transitions the device to an operational state, ready to execute dosing commands.

#### Supported Commands

The `LiquidDoser` node supports a variety of commands as outlined in the following table:

| Command | Value | Optional | Response Format       |
| ------- | ----- | -------- | --------------------- |
| `D`     | ml    | min      | `*OK`                |

Refer to the node's code comments and inline documentation for additional command handling details and error management.

#### Example Usage

To use the `LiquidDoser` node, ensure the node is running and configured with the necessary parameters. You can initiate a dosing command using the `start_liquid_dosing` service call as follows:

```bash
ros2 service call /start_liquid_dosing alex_interfaces/srv/LiquidDoserCommand "{
  job_id: <optional>,
  command: 'D', 
  value: 5.0,    # amount in ml
}"
```

In this example:

- job_id: Assigns a job ID to the operation for tracking.
- command: The dosing command to send, in this case, D for a dispensing command.
- value: Sets the amount to be dispensed (in this example, 5.0 ml).
- optional: An optional parameter; 

### 4. LiquidLevel ROS2 Node

The `LiquidLevel` ROS2 node calculates and publishes the liquid volume in a container based on sensor data received via a serial interface. It supports cuboid and cylindrical containers, and provides services to query the current or average liquid volume.

#### Key Features

- **Container Types**: Supports `cuboid` and `cylinder`.
- **Volume Calculation**: 
  - **Cuboid**: Volume = Length × Width × Liquid Height
  - **Cylinder**: Volume = π × Radius² × Liquid Height
- **Parameters**:
  - `max_height`, `length`, `width`, `radius`, `sensor_offset`
  - `critical_ml`: Critical volume threshold (in milliliters).
  - `deque_maxlen`: Max number of liquid level readings to average.
- **Subscriptions**: Listens to `from_serial` for liquid level data (formatted as `LL:<value>`).
- **Services**: `start_liquid_level_reading` allows querying the latest or average liquid volume.
- **Publishing**: Periodically publishes the liquid volume to the `liquid_volume` topic.

#### Serial Data
Ensure the serial device sends liquid level data in the format `LL:<value>`, e.g., `LL:150`.

### Example usage

- **Get Latest Volume**:
```bash
ros2 service call /start_liquid_level_reading alex_interfaces/srv/LiquidLevelCommand "{command: 1}"
```


### 5. WeightBalance ROS2 Node

The `WeightBalance` ROS2 node manages a serial device for weight measurements. It stabilizes weight readings and provides them via a ROS2 service and topic.

#### Features

- **Device Lifecycle**: Configures and activates the serial device using lifecycle services.
- **Weight Measurement**: Waits for stable weight data (configurable duration).
- **Services**: 
  - **`start_weight_balance`**: Requests the latest or stable weight data.
- **Topic**: Publishes weight status on `weight_data`.

#### Parameters

- **`serial_node_name`**: Serial device name.
- **`weight_stabilizing_duration`**: Time to wait for stable readings (seconds).

#### Example Usage

- **Latest Weight**:
  ```bash
  ros2 service call /start_weight_balance alex_interfaces/srv/WeightCommand "{mode: 2}"

#### NOTE:

While connecting to the serial port CH340 chip of RS232-USB converter, faced some issue. 

[ISSUE](https://github.com/arduino/arduino-ide/issues/1788)

Solution: 
```shell
sudo apt-get autoremove brltty
```

### 6. ph_meter_peripheral

The ph_meter_peripheral node is responsible for interfacing with a pH sensor device to measure and publish pH values. The node communicates with the device over an FTDI serial connection and supports starting/stopping pH measurement sessions using ROS2 services. It also includes automatic reconnection logic to handle disconnections.

#### Parameter:
- **ph_meter_serial_number**: Serial number of the device.

#### Topics:
- **ph_reading**: Publishes pH value and optional job_id.

#### Service:
- **start_ph_reading**: Start/stop pH readings.

#### Features:
- Auto-reconnect on disconnection.
- Reports -1.0 if the device is unavailable.

#### NOTE:

**Installation**:  [Atlas Installation Documentation](https://files.atlas-scientific.com/pi_sample_code.pdf)
