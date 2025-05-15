#!/bin/bash
# Define the node name and YAML path
NODE_NAME="serial_interface_atlas_liquid_doser"
YAML_PATH="$(ros2 pkg prefix alex_externals)/share/alex_externals/params/alex_external.yaml"

# Run the Python script to find the serial port, capturing both stdout and stderr
PORT=$(ros2 run alex_utilities get_port "${YAML_PATH}" "${NODE_NAME}" 2>&1)

# Check if the output contains the word "Error"
if [[ "$PORT" == *"Error"* || -z "$PORT" ]]; then
    echo "Error: Could not find the serial port or an issue occurred in the Python script."
    echo "Details: $PORT"
    exit 1
fi

echo "Found port: $PORT"
echo "Uploading the code"

# Uncomment this line to upload the code to the Arduino
make upload PORT=$PORT
