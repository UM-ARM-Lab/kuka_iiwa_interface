# Victor Sim Hardware

This package provides a simulated hardware interface for the Victor dual-arm robot. It replaces the real hardware interface with a simulation that provides the same grouped command interfaces needed by ROS2 Control and MoveIt.

## Features

- **Complete Hardware Interface**: Provides all joint position, velocity, and effort interfaces for both arms
- **Grouped Command Interfaces**: Critical grouped interfaces like `left/joint_position`, `right/joint_position`, `left/joint_impedance`, `right/joint_impedance` that were missing from mock hardware
- **Multiple Control Modes**: Supports joint position, joint impedance, cartesian pose, and cartesian impedance control
- **Force-Torque Simulation**: Simulated force-torque sensors for each arm
- **Gripper Support**: Simulated gripper state interfaces
- **Python API**: High-level Python interface for external scripts to interact with robot state

## Architecture

The package consists of:

1. **C++ Hardware Interface** (`VictorSimHardwareInterface`): Core hardware interface plugin that provides all the grouped command interfaces
2. **Python API** (`VictorRobotStateAPI`): High-level Python interface for reading robot state and sending commands
3. **Examples**: Demonstration scripts showing how to use the Python API

## Installation

Build the package in your ROS2 workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select victor_sim_hardware
source install/setup.bash
```

## Usage

### 1. Using with ROS2 Control

Replace the mock hardware in your XACRO configuration:

```xml
<ros2_control name="victor" type="system">
  <hardware>
    <!-- Replace mock_components/GenericSystem with: -->
    <plugin>victor_sim_hardware/VictorSimHardwareInterface</plugin>
  </hardware>
  <!-- ... joint definitions ... -->
</ros2_control>
```

### 2. Using the Python API

```python
import rclpy
from victor_sim_hardware import VictorRobotStateAPI

# Initialize ROS2
rclpy.init()

# Create robot API
robot = VictorRobotStateAPI()

# Wait for robot state
robot.wait_for_state_update()

# Read joint positions
left_joints = robot.get_joint_positions('left')
print(f"Left arm joints: {left_joints}")

# Send joint commands
new_positions = {
    'left_arm_joint1': 0.1,
    'left_arm_joint2': 0.2,
    # ... other joints
}
robot.send_joint_position_command('left', new_positions)

# Switch control modes
robot.set_control_mode('left', 'cartesian_pose')

# Read cartesian pose
pose = robot.get_cartesian_pose('left')
if pose:
    position, orientation = pose
    print(f"Left arm pose: {position}, {orientation}")

# Clean up
robot.destroy_node()
rclpy.shutdown()
```

### 3. Running Examples

```bash
# Run the simple example
ros2 run victor_sim_hardware victor_robot_api_example

# Or run directly
cd ~/ros2_ws/src/kuka_iiwa_interface/victor_sim_hardware/examples
python3 simple_example.py
```

## Key Differences from Mock Hardware

The main advantage over `mock_components/GenericSystem` is that this hardware interface provides the grouped command interfaces that real hardware automatically exposes:

- `left/joint_position` - Grouped joint position commands for left arm
- `right/joint_position` - Grouped joint position commands for right arm  
- `left/joint_impedance` - Grouped joint impedance commands for left arm
- `right/joint_impedance` - Grouped joint impedance commands for right arm
- `left/cartesian_pose` - Cartesian pose commands for left arm
- `right/cartesian_pose` - Cartesian pose commands for right arm
- `left/cartesian_impedance` - Cartesian impedance commands for left arm  
- `right/cartesian_impedance` - Cartesian impedance commands for right arm

These grouped interfaces allow controllers to switch between different control modes for each arm independently, which is essential for complex dual-arm manipulation tasks.

## API Reference

### VictorRobotStateAPI Methods

#### State Reading Methods:
- `get_joint_positions(arm='both')` - Get current joint positions
- `get_joint_velocities(arm='both')` - Get current joint velocities  
- `get_joint_efforts(arm='both')` - Get current joint efforts/torques
- `get_cartesian_pose(arm)` - Get current cartesian pose of end-effector
- `get_force_torque(arm)` - Get current force-torque sensor reading
- `get_gripper_state(arm)` - Get current gripper state

#### Command Methods:
- `send_joint_position_command(arm, positions)` - Send joint position command
- `send_cartesian_pose_command(arm, position, orientation)` - Send cartesian pose command  
- `set_control_mode(arm, mode)` - Set control mode for an arm

#### Utility Methods:
- `wait_for_state_update(timeout=5.0)` - Wait for robot state to be updated

## Control Modes

Supported control modes for each arm:
- `'joint_position'` - Joint space position control
- `'joint_impedance'` - Joint space impedance control
- `'cartesian_pose'` - Cartesian space pose control
- `'cartesian_impedance'` - Cartesian space impedance control

## Troubleshooting

1. **Hardware interface not found**: Make sure the package is built and sourced
2. **Python import errors**: Ensure Python dependencies are installed
3. **No robot state received**: Check that the hardware interface is properly loaded and joint_states are being published

## Contributing

This package was created to solve the missing grouped command interfaces issue in the Victor robot simulation. Feel free to extend it with additional features as needed.
