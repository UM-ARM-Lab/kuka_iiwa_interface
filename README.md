# kuka_iiwa_interface
Developmental controllers for the IIWA 7 robot, including a Catkinized version of the Kuka FRI SDK

# Dependencies
- [arc_utilities](https://github.com/UM-ARM-Lab/arc_utilities)
- moveit

## Cartesian Pose Mode
Cartesian pose commands should target poses for the `victor_[left/right]_gripper_palm_surface_kuka` frame, relative to `victor_[left/right]_arm_world_frame_kuka`

## MedSetup Mode
When switch to this mode, the robot starts the cyclical brake test, and reset the 24 hours cyclic timer. Still on working work, though now it already works.