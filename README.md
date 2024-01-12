# kuka_iiwa_interface
Developmental controllers for the IIWA 7 robot

# Dependencies

See `repos.yaml` for dependencies that need to be cloned and build fom source.

For other dependencie, use `rosdep install -r -y --ignore-src --from-paths src/` to install them.

## Cartesian Pose Mode
Cartesian pose commands should target poses for the `victor_[left/right]_gripper_palm_surface_kuka` frame, relative to `victor_[left/right]_arm_world_frame_kuka`
