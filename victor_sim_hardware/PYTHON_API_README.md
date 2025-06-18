# Victor Sim Hardware - Python API

This document describes how to use the Python Robot State API to create simulators that communicate with the `victor_sim_hardware` interface.

## Overview

The `victor_sim_hardware` package provides a ROS2-based hardware interface for the Victor robot that communicates with external Python simulators via ROS topics. This allows you to create custom physics simulators or mock robot behaviors while maintaining compatibility with existing Victor controllers.

## Architecture

The system uses a dual-topic approach:

### Standard Victor API Topics (for controllers)
- `/victor/left_arm/motion_status` - Robot state published to controllers
- `/victor/right_arm/motion_status` - Robot state published to controllers  
- `/victor/left_arm/gripper_command` - Gripper commands from controllers
- `/victor/right_arm/gripper_command` - Gripper commands from controllers
- `/victor/left_arm/gripper_status` - Gripper status published to controllers
- `/victor/right_arm/gripper_status` - Gripper status published to controllers

### Simulator Bridge Topics (for hardware interface communication)
- `/victor_sim_bridge/left/motion_command` - Joint position commands from hardware interface (JointValueQuantity)
- `/victor_sim_bridge/right/motion_command` - Joint position commands from hardware interface (JointValueQuantity)
- `/victor_sim_bridge/left/motion_status` - Robot state to hardware interface (MotionStatus)
- `/victor_sim_bridge/right/motion_status` - Robot state to hardware interface (MotionStatus)
- `/victor_sim_bridge/left/gripper_command` - Gripper commands to hardware interface (Robotiq3FingerCommand)
- `/victor_sim_bridge/right/gripper_command` - Gripper commands to hardware interface (Robotiq3FingerCommand)
- `/victor_sim_bridge/left/gripper_status` - Gripper status to hardware interface (Robotiq3FingerStatus)
- `/victor_sim_bridge/right/gripper_status` - Gripper status to hardware interface (Robotiq3FingerStatus)

## Python API Usage

### Basic Setup

```python
import rclpy
from victor_sim_hardware.robot_state_api import create_victor_simulator

# Create the simulator API
simulator = create_victor_simulator()

# Get arm APIs
left_arm = simulator.get_left_arm()
right_arm = simulator.get_right_arm()
```

### Reading Commands (Polling Method - Recommended)

```python
import time
import numpy as np

# Main simulation loop
while rclpy.ok():
    # Poll for latest commands from controllers
    left_motion_cmd = left_arm.get_latest_motion_command()
    if left_motion_cmd is not None:
        # left_motion_cmd is a numpy array of 7 joint positions
        print(f"Left arm command: {left_motion_cmd}")
        # Use these joint targets in your physics simulation
        
    left_gripper_cmd = left_arm.get_latest_gripper_command()
    if left_gripper_cmd is not None:
        # left_gripper_cmd is a numpy array [finger_a, finger_b, finger_c, scissor]
        print(f"Left gripper command: {left_gripper_cmd}")
        
    # Update your physics simulation here...
    # Then update robot state and publish
    
    left_arm.publish_motion_status()
    left_arm.publish_gripper_status()
    
    time.sleep(0.001)  # 1000 Hz
```

### Updating Robot State

```python
# Update joint states (from your physics simulation)
joint_positions = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]  # 7 joint angles in radians
joint_velocities = [0.0] * 7  # 7 joint velocities in rad/s
joint_efforts = [0.0] * 7     # 7 joint efforts in Nm
external_torques = [0.0] * 7  # 7 external torques in Nm

left_arm.set_joint_positions(joint_positions)
left_arm.set_joint_velocities(joint_velocities)
left_arm.set_joint_efforts(joint_efforts)
left_arm.set_external_torques(external_torques)

# Update cartesian pose (from forward kinematics)
position = [0.5, 0.2, 0.3]  # [x, y, z] in meters
orientation = [0, 0, 0, 1]  # [x, y, z, w] quaternion
left_arm.set_cartesian_pose(position, orientation)

# Update gripper state
finger_a_pos = 0.5  # 0.0 to 1.0
finger_b_pos = 0.5  # 0.0 to 1.0  
finger_c_pos = 0.5  # 0.0 to 1.0
scissor_pos = 0.0   # -1.0 to 1.0
left_arm.set_gripper_positions(finger_a_pos, finger_b_pos, finger_c_pos, scissor_pos)

# Publish state to ROS (call this regularly, e.g., at 100-1000 Hz)
left_arm.publish_motion_status()
left_arm.publish_gripper_status()
```

### Complete Example

```python
#!/usr/bin/env python3
import time
import numpy as np
import rclpy
from victor_sim_hardware.robot_state_api import create_victor_simulator

def main():
    # Initialize the simulator
    simulator = create_victor_simulator()
    
    try:
        # Set up callbacks
        def left_motion_callback(msg):
            cmd_pos = msg.commanded_joint_position
            print(f"Left arm command: [{cmd_pos.joint_1:.3f}, {cmd_pos.joint_2:.3f}, ...]")
        
        simulator.left_arm.set_motion_command_callback(left_motion_callback)
        
        # Main simulation loop
        rate_hz = 100  # 100 Hz update rate
        while rclpy.ok():
            # Your physics simulation goes here
            current_time = time.time()
            
            # Example: sinusoidal motion
            positions = [0.1 * np.sin(current_time + i) for i in range(7)]
            
            # Update robot state
            simulator.left_arm.set_joint_positions(positions)
            simulator.left_arm.set_joint_velocities([0.0] * 7)
            simulator.left_arm.set_joint_efforts([0.0] * 7)
            simulator.left_arm.set_external_torques([0.0] * 7)
            
            # Publish state
            simulator.left_arm.publish_motion_status()
            simulator.left_arm.publish_gripper_status()
            
            time.sleep(1.0 / rate_hz)
    
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        simulator.stop()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
```

## Running with Victor Controllers

1. **Start the hardware interface:**
   ```bash
   ros2 launch victor_bringup victor_sim.launch.py
   ```

2. **Run your Python simulator:**
   ```bash
   cd /path/to/your/simulator
   python3 my_simulator.py
   ```

3. **Launch controllers:**
   ```bash
   ros2 launch victor_controllers victor_controllers.launch.py
   ```

## API Reference

### VictorSimulatorAPI

Main class that manages both arms and ROS communication.

**Methods:**
- `get_left_arm()` → `ArmAPI` - Get left arm interface
- `get_right_arm()` → `ArmAPI` - Get right arm interface
- `start()` - Start ROS callback processing
- `stop()` - Stop and cleanup

### ArmAPI

Interface for a single arm (left or right).

**State Setting Methods:**
- `set_joint_positions(positions: List[float])` - Set 7 joint positions (radians)
- `set_joint_velocities(velocities: List[float])` - Set 7 joint velocities (rad/s)
- `set_joint_efforts(efforts: List[float])` - Set 7 joint efforts (Nm)
- `set_external_torques(torques: List[float])` - Set 7 external torques (Nm)
- `set_cartesian_pose(position: List[float], orientation: List[float])` - Set end-effector pose
- `set_gripper_positions(finger_a, finger_b, finger_c, scissor)` - Set gripper finger positions

**State Getting Methods:**
- `get_joint_positions()` → `np.ndarray` - Get current joint positions
- `get_joint_velocities()` → `np.ndarray` - Get current joint velocities
- `get_joint_efforts()` → `np.ndarray` - Get current joint efforts
- `get_external_torques()` → `np.ndarray` - Get current external torques
- `get_cartesian_pose()` → `Pose` - Get current cartesian pose

**Command Handling:**
- `set_motion_command_callback(callback)` - Set callback for motion commands
- `set_gripper_command_callback(callback)` - Set callback for gripper commands
- `get_latest_motion_command()` → `Optional[MotionStatus]` - Get last motion command
- `get_latest_gripper_command()` → `Optional[Robotiq3FingerCommand]` - Get last gripper command

**Publishing:**
- `publish_motion_status()` - Publish current robot state
- `publish_gripper_status()` - Publish current gripper state

## Message Types

The API uses standard Victor hardware interface message types:
- `victor_hardware_interfaces/msg/MotionStatus` - Robot joint and cartesian state
- `victor_hardware_interfaces/msg/Robotiq3FingerCommand` - Gripper commands
- `victor_hardware_interfaces/msg/Robotiq3FingerStatus` - Gripper status

See the Victor hardware interfaces package for detailed message definitions.

## Examples

See the `examples/` directory for complete working examples:
- `simple_simulator_example.py` - Basic physics simulation with PD control
- `simulator_bridge_example.py` - Example showing direct integration patterns

## Troubleshooting

**Issue: Import errors**
- Make sure the package is built: `colcon build --packages-select victor_sim_hardware`
- Source the workspace: `source install/setup.bash`

**Issue: No motion commands received**
- Verify controllers are running and publishing to `/victor/{side}_arm/motion_command`
- Check that the hardware interface is active

**Issue: Robot state not updating in RViz**
- Ensure `publish_motion_status()` is called regularly (100+ Hz recommended)
- Verify joint positions are within valid ranges
- Check that message timestamps are current

**Issue: Topics not visible**
- Use `ros2 topic list` to verify topics are being published
- Check topic types with `ros2 topic info /victor_sim_bridge/left/motion_status`

## Performance Recommendations

- Call `publish_motion_status()` at 100-1000 Hz for smooth operation
- Use efficient physics simulation libraries (PyBullet, MuJoCo, etc.)
- Consider running physics simulation in separate thread from ROS callbacks
- Monitor CPU usage and adjust update rates as needed
