
# SerialInterfaceLifecycleNode

This ROS2 lifecycle node, `SerialInterfaceLifecycleNode`, manages a serial connection with the ability to reconnect automatically when the connection is lost. The node transitions through different lifecycle states, providing structured stages for initialization, activation, deactivation, cleanup, and shutdown.

## Running the Node

To run the node from the terminal:

```bash
ros2 run alex_externals serial_interface_lifecycle_node
```

## Launching with Arguments

You can customize node parameters by passing arguments at launch. For example:

```bash
ros2 run alex_externals serial_interface_lifecycle_node --ros-args -p device_port:="/dev/ttyACM0" -p baud_rate:=9600
```

- `device_port`: The serial number of the device to connect to.
- `baud_rate`: Serial connection baud rate (default is 115200).
- Additional parameters include `serial_number` and `firmware_version`.
> **NOTE:** `device_port` is given priority over `serial_number` for connecting.

## Changing Node Lifecycle States

The `SerialInterfaceLifecycleNode` supports transitions between states that control its behavior and the connection. You can change the node state using the `ros2 lifecycle` command:

1. **Configure the node**:
   ```bash
   ros2 lifecycle set /serial_interface_lifecycle_node configure
   ```
   Sets up the node with the provided parameters, opening a serial connection if possible.

2. **Activate the node**:
   ```bash
   ros2 lifecycle set /serial_interface_lifecycle_node activate
   ```
   Activates the node, enabling data to be read from and written to the serial connection.

3. **Deactivate the node**:
   ```bash
   ros2 lifecycle set /serial_interface_lifecycle_node deactivate
   ```
   Temporarily stops the serial communication without resetting parameters.

4. **Cleanup the node**:
   ```bash
   ros2 lifecycle set /serial_interface_lifecycle_node cleanup
   ```
   Cleans up resources, closing the serial connection and resetting parameters.

5. **Shutdown the node**:
   ```bash
   ros2 lifecycle set /serial_interface_lifecycle_node shutdown
   ```
   Shuts down the node, releasing all resources.

## Interacting with Topics

The node uses two primary topics for data transmission:

- **to_serial** (`/to_serial`): Used to send data from ROS to the serial device.
- **from_serial** (`/from_serial`): Used to receive data from the serial device to ROS.

### 1. Publishing Data to `to_serial`

To send data to the serial device, you can publish a message on the `to_serial` topic. The message type is `SerialMessage`, and it contains a `data` field for the content to be sent.

Example command to publish a message to `to_serial`:

```bash
ros2 topic pub /to_serial alex_interfaces/msg/SerialMessage "{data: 'Hello, serial device!'}"
```

### 2. Echoing Data from `from_serial`

To view incoming data from the serial device, use the following command to echo messages from the `from_serial` topic:

```bash
ros2 topic echo /from_serial
```

Messages published on `from_serial` contain:
- `state`: The connection state (`CONNECTED`, `DISCONNECTED`, etc.)
- `data`: The data received from the serial connection
