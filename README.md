# How do I make the robot move?

This depends on the type of motion you want.
You should also consider the `Control Mode` you want to use (for the Kuka there are many).
The main use cases are:

### Traditional Motion Planning

You want to plan a collision free trajectory to a joint configuration, and execute open loop.
Variations on this include planning to a given pose.
Generally, you aren't doing any feedback control, you just want to move the robot blindly to a given configuration.
You probably are also intending to move a large distance (many degrees/centimeters, or much more).
The resulting motion should be very smooth.
Do not try to call this repeatedly in a loop quickly!

**Solution**:
Use MoveIt with the `ompl` planning pipeline.
This works with `JOINT_POSITION` or `JOINT_IMPEDANCE` control modes, but not `CARTESIAN_POSE` or `CARTESIAN_IMPEDANCE`.

### Cartesian Motion Planning

You want to move in a "straight line" or some cartesian motion, also open loop, possibly also collision free.
Similar assumptions/notes to the above case.

**Solution**:
Use MoveIt with the `pilz` planning pipeline.
This again only works with `JOINT_POSITION` or `JOINT_IMPEDANCE` control modes.

### Closed-Loop Joint Position Control

You want to do MPC, visual servoing, RL, teleop, or behavior cloning, with joint position actions.
You don't expect the robot to move very far, and your planner or controller with be responsible for ensuring the robot doesn't collide with itself or
anything it could damage.

**Solution**:
Send `MotionCommand` messages via ROS or LCM to each arm.
You must be sure to fill out *all* the fields in the `MotionCommand` message, and smoothness is dependent on your commands and the control mode
parameters.
This works with `CARTESIAN_POSE` or `CARTESIAN_IMPEDANCE` control modes.

### Closed-Loop Cartesian Control

The same as above, but with cartesian actions.

**Solution**:
Same as above.
The cartesian control modes are known to jerk near singularities and violate joint limits, so your planner or controller should try very hard to avoid
this.

### Closed-Loop Joint Velocity Control

You cannot do this with the Kuka robots -- at least not easily in the general case.
The only workaround here is to use Joint position/impedance modes and send joint position commands carefully calculated and carefully timed to try to
move the robot smoothly.
This may also require carefully setting the control mode parameters (specifically, `joint_path_execution_params.joint_relative_velocity`
or `joint_path_execution_params.joint_relative_acceleration`).
Honestly, it is probably not worth your time to try to do this.

## MoveItPy Background Information

This package relies mostly on MoveIt's Python API to plan trajectories.
MoveIt Python API is a wrapper around MoveIt's C++ API, and it is not as complete as the C++ API.
For planning, we primarily rely on python bindings to the `moveit_cpp` package.
The `moveit_cpp` package is ROS-ified C++ code that wraps up the core moveit planning API.
It was created as a Google Summer of Code project and may not receive much maintenance by the core MoveIt developers, and may not be as future-proof.
As of Feb 2024, there are some gotcha's with the python API:

1. Your MoveIt configuration package needs to have a `moveit_cpp.yaml` file, which may not be generated when you use the MoveIt Setup Assistant!
   See [this documentation](https://moveit.picknik.ai/main/doc/how_to_guides/moveit_configuration/moveit_configuration_tutorial.html) for more
   information on MoveIt configuration files.

2. Use `MoveItConfigsBuilder` to load all the yaml files in your MoveIt configuration package.
   This function works with almost no arguments if you follow all the conventions for naming files, otherwise you may need to specify the file names.
   You need to explicitly configure `moveit_cpp`, for example:
      ```python
      from ament_index_python import get_package_share_directory
      from moveit_configs_utils import MoveItConfigsBuilder
      moveit_config_builder = MoveItConfigsBuilder('victor')
      moveit_config_builder.moveit_cpp(file_path=get_package_share_directory("victor_moveit_config") + "/config/moveit_cpp.yaml")
      ```

3. When you instantiate a `MoveItPy` object, it will instaniate a `moveit_cpp::MoveItCpp` instance, which will create a `rclcpp::Node`.
   However, as far as I am aware, calling python functions to plan using `MoveItPy` does not make actually use any ROS topics/services, it directly
   calls C++ functions.

# Dependencies

See `repos.yaml` for dependencies that need to be cloned and build fom source.
Use `vcs import < kuka_iiwa_interface/repos.yaml` to clone them.

For other dependencies, use `rosdep install -r -y --ignore-src --from-paths src/` to install them.
