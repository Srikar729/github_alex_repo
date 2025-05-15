#!/usr/bin/env python3

import yaml
import sys
from serial.tools import list_ports
from pathlib import Path


def load_yaml(yaml_file_path):
    """Load and return the content of a YAML file."""
    try:
        with yaml_file_path.open('r') as file:
            return yaml.safe_load(file)
    except FileNotFoundError:
        print(f"Error: YAML file '{yaml_file_path}' not found.", file=sys.stderr)
        sys.exit(1)
    except yaml.YAMLError as e:
        print(f"Error: Failed to parse YAML file. {e}", file=sys.stderr)
        sys.exit(1)


def get_serial_number(yaml_content, node_name):
    """Retrieve the serial number for the given node name from the YAML content."""
    try:
        return yaml_content[f"/{node_name}"]["ros__parameters"]["serial_number"]
    except KeyError:
        print(f"Error: Could not find the serial number for node '{node_name}' in the YAML file.", file=sys.stderr)
        sys.exit(1)


def find_arduino_port(serial_number):
    """Find the Arduino port matching the given serial number."""
    com_ports = list_ports.comports()
    arduino_port, *_ = [com_port for com_port in com_ports if com_port.serial_number == serial_number] or [None]
    
    if not arduino_port:
        print(f"Error: No COM port found for serial number '{serial_number}'", file=sys.stderr)
        sys.exit(1)
    
    return arduino_port.device


def main(argv=None):
    argv = argv or sys.argv
    if len(argv) < 3:
        print("Usage: script.py <yaml_file> <node_name>", file=sys.stderr)
        sys.exit(1)
    
    yaml_file_path = Path(argv[1])
    node_name = argv[2]
    
    # Load YAML content
    yaml_content = load_yaml(yaml_file_path)
    
    # Get serial number for the node
    serial_number = get_serial_number(yaml_content, node_name)
    
    # Find the Arduino port
    arduino_port_device = find_arduino_port(serial_number)
    
    # Print the device (COM port) to stdout
    print(arduino_port_device)


if __name__ == "__main__":
    main(sys.argv)
