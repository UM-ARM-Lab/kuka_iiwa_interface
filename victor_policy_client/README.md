# Victor Policy Client

A lightweight ROS2 package for interfacing with the Victor robot using policy-based control. This package provides a minimal dependency client for executing robot policies without requiring the full victor_python stack.

## Overview

The `victor_policy_client` package contains:

- `VictorPolicyClient`: Core client for communicating with Victor robot policy servers
- `VictorEnvBase`: Abstract base class for policy environments
- Hardware interface utilities for numpy/torch integration
- Data utilities for efficient data handling

## Dependencies

### ROS Dependencies
- `rclpy`: ROS2 Python client library
- `victor_hardware_interfaces`: Victor-specific hardware interface messages
- `geometry_msgs`: Standard ROS geometry messages
- `std_msgs`: Standard ROS messages
- `ros2_numpy`: NumPy integration for ROS2

### Python Dependencies
- `numpy`: Numerical computing
- `torch`: PyTorch for tensor operations

## Installation

This package is designed to be built as a standard ROS2 package using colcon:

```bash
cd /path/to/your/ros2_workspace
colcon build --packages-select victor_policy_client
source install/setup.bash
```

## Usage

### Basic Client Usage

```python
from victor_policy_client.victor_policy_client import VictorPolicyClient
import rclpy

rclpy.init()
client = VictorPolicyClient('my_client')

# Check if server is available
if client.check_server_availability():
    # Use the client for policy execution
    pass

client.destroy_node()
rclpy.shutdown()
```

### Console Scripts

The package provides the following console scripts:

- `victor_policy_client`: Basic client test and demonstration

### Examples

Example usage can be found in the `examples/policy_client_examples.py` file, which is installed to the share directory.

## Architecture

The package is designed to be lightweight and focused on policy client functionality, separating it from the heavier dependencies in the main victor_python package.

## License

MIT License
