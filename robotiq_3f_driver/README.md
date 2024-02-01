# ROS2 Robotiq 3F Gripper Serial Driver

This package contains a C++ ROS Node that talk to the Robotiq 3F Gripper via Serial, and exposes a ROS API

## ROS API

### Launch files

```
# Launch the driver and the robot_state_publisher.
# This is the main launch file to use.
ros2 launch robotiq_3f_description robotiq_3f_gripper.launch.xml

# Just show the gripper in its default joint configuration in rviz.
ros2 launch robotiq_3f_description visualize_description.launch.xml

```

### Publishers

Publishes the full joint states, so the URDF can show the state of the gripper. This requires the `robot_state_publisher` to be running.
```
name: joint_states
type: sensor_msgs/msg/JointState
```

### Subscribers

For opening and closing the gripper with the most control, publish commands to this topic:
```
name: independent_control_command
type: robotiq_3f_interfaces/msg/IndependentControlCommand
```

For a simple command API, you can use this one.
The exact grasp also depends on the grasping mode, which can be changed with the service below.
```
name: simple_control_command
type: robotiq_3f_interfaces/msg/SimpleControlCommand
```

### Service Servers

Change the grasping mode, which effects the simple control command and the action server.
```
name: change_grasping_mode
type: robotiq_3f_interfaces/srv/ChangeGraspingMode
```

### Action Servers

The simple way to open and close the gripper is to use the action server.
This is also a standard action server, so other packages like MoveIt know how to call it.
```
name: gripper_cmd
type: control_msgs/action/GripperCommand
```

## Code Structure

The hardware interface is implemented in the `robotiq_3f_driver` package, and the main executable is the ROS Node `robotiq_3f_driver_node`, also in the hardware package.
There is also a custiom transmission, which makes from the actuator states (4 motors) to an approximate joint state for the full articulated gripper (11 joints). This is then published on the `joint_states` topic. The URDF describes the gripper with 3 revolute joints per finger, and we simulate the underactuated kinematics using the transmission.

# Launch the driver and the robot_state_publisher
ros2 launch robotiq_3f_description robotiq_3f_gripper.launch.xml
