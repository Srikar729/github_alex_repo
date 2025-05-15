# Device Config Samples

Here are some of the device configuration that needs to be added to thingsboard for each devices.

> [!IMPORTANT]  
> The serial number and other parameters need to be updated accordingly

### Zed Mini Camera:

```json
{
  "device_type": "zed-camera",
  "external_nodes": [
    {
      "node_name": "zed_camera_image"
    }
  ]
}
```

### Liquid Dispensors:

```json
{
  "device_type": "liquid-dispensor",
  "external_nodes": [
    {
      "node_name": "atlas_liquid_doser",
      "prefix": "",
      "parameters": {
        "max_acknowledgement_duration": 10,
        "max_completion_duration": 60
      }
    },
    {
      "node_name": "liquid_level",
      "prefix": "",
      "parameters": {
        "container_type": "cuboid",
        "max_height": 15.000001,
        "length": 13.5,
        "width": 8.8,
        "radius": 0.0001,
        "sensor_offset": 0.3,
        "critical_ml": 500,
        "deque_maxlen": 10
      }
    },
    {
      "node_name": "serial_interface_lifecycle",
      "prefix": "",
      "parameters": {
        "serial_number": "14101",
        "baud_rate": 115200,
      }
    }
  ]
}
```

### Atlas pH meter
```json
{
  "device_type": "pH-meter",
  "external_nodes": [
    {
      "node_name": "atlas_ph_probe",
      "prefix": "ph",
    },
    {
      "node_name": "serial_interface_lifecycle",
      "prefix": "ph",
      "parameters": {
        "baud_rate": 9600,
        "serial_number": "DP05OU2I",
        "read_type": "utf-8",
        "timeout": 1,
        "read_terminator": "\r",
        "write_terminator": "\r"
      }
    },
    {
      "node_name": "stepper_actuator",
      "prefix": "actuator",
      "parameters": {
        "max_stroke_length": 370,
        "max_acknowledgement_duration": 10,
        "max_completion_duration": 60
      }
    },
    {
      "node_name": "serial_interface_lifecycle",
      "prefix": "actuator",
      "parameters": {
        "baud_rate": 115200,
        "serial_number": "12148509806178206400",
        "read_type": "utf-8",
        "timeout": 1
      }
    }
  ],
  "controller": {
    "node_name": "actuated_ph_control",
    "prefix": "",
    "parameters": {
      "max_measureing_delay": 60,
      "has_temperature": false
    }
  }
}
```

### Atlas pH meter with temperature
```json
{
  "device_type": "pH-meter",
  "external_nodes": [
    {
      "node_name": "atlas_ph_probe",
      "prefix": "ph",
      "parameters": {}
    },
    {
      "node_name": "serial_interface_lifecycle",
      "prefix": "ph",
      "parameters": {
        "baud_rate": 9600,
        "serial_number": "DP05OU2I",
        "read_type": "utf-8",
        "timeout": 1,
        "read_terminator": "\r",
        "write_terminator": "\r"
      }
    },
    {
      "node_name": "atlas_temperature_probe",
      "prefix": "temperature",
    },
    {
      "node_name": "serial_interface_lifecycle",
      "prefix": "temperature",
      "parameters": {
        "baud_rate": 9600,
        "serial_number": "DQ01FTHS",
        "read_type": "utf-8",
        "timeout": 1,
        "read_terminator": "\r",
        "write_terminator": "\r"
      }
    },
    {
      "node_name": "stepper_actuator",
      "prefix": "actuator",
      "parameters": {
        "max_stroke_length": 370,
        "max_acknowledgement_duration": 10,
        "max_completion_duration": 60
      }
    },
    {
      "node_name": "serial_interface_lifecycle",
      "prefix": "actuator",
      "parameters": {
        "baud_rate": 115200,
        "serial_number": "12148509806178206400",
        "read_type": "utf-8",
        "timeout": 1
      }
    }
  ],
  "controller": {
    "node_name": "actuated_ph_control",
    "prefix": "",
    "parameters": {
      "max_measureing_delay": 60,
      "has_temperature": true
    }
  }
}
```

### Ohaus Weight Balance
<!-- TODO: In progress -->
```json
{
  "device_type": "weight-balance",
  "external_nodes": [
    {
      "node_name": "weight_balance",
      "prefix": "weight-balance",
      "parameters": {
        "weight_stabilizing_duration": 10
      }
    },
    {
      "prefix": "weight-balance",
      "node_name": "serial_interface_lifecycle",
      "parameters": {
        "serial_number": "CPDWe12CJ06",
        "baud_rate": 9600,
        "timeout": 5,
        "read_terminator": "\r"
      }
    }
  ],
  "controller": {
    "node_name": "automated_weight_balance"
  }
}
```
