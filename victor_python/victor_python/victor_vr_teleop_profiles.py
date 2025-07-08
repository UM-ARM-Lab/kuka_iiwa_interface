"""
This file contains the teleoperation profiles for the victor robot

To make your own profile, inherit from `VictorTeleopProfile` and override the necessary attributes.
The profiles are registered in the `teleop_profile_dict` dictionary with the profile name as the key.
You can then use the profile by its name in the teleoperation code. 
"""
import numpy as np

# The following parts 
# Register profiles for teleop for different configurations
teleop_profile_dict = {}

class VictorTeleopProfile:
    use_arms=['left', 'right']
    # Sensitivity sttings
    position_sensitivity=1.0
    orientation_sensitivity=1.0
    # Controller settings
    target_controller="impedance_controller"
    use_filter=True
    # Joint settings
    # If has init joints, put them in a list as use_arms list
    init_joints={}
    max_ctrl_delay=0.5  # Maximum delay for controller commands in seconds
    
    # Usability rotation for the controllers
    # List corresponding to use_arms
    usability_rotation = {
        "left": [0.0,0.0,0.0], # left
        "right":[0.0,0.0,0.0]  # right
    }

    # Gripper settings
    # List of lists of 4 values (a,b,c,scissor) for gripper
    # Gripper will be initialized at `init_gripper_state`, the index for which list to use
    gripper_keypoints = [
        [0.0, 0.0, 0.0, 0.0],       # fully open
    ]
    init_gripper_state=0

    # Viewport tracker
    # This can be useful for moving camera around a simulator
    use_viewport_tracker = False
    viewport_position_sensitivity = 1.0
    viewport_orientation_sensitivity = 1.0
    
    def __init__(self):
        self.use_left = 'left' in self.use_arms
        self.use_right = 'right' in self.use_arms

class VictorTeleopRealRobotProfile(VictorTeleopProfile):
    position_sensitivity = 0.5
    ctrl_mode = "trackpad_rot"
    init_joints = {
        "left": [
            1.6022122533307945,
            -0.9978108494251933,
            -0.7075040305616351,
            -1.8461418367034075,
            -0.5849508065575095,
            0.8936085770212594,
            1.5204164680326602,
        ], 
        "right": [
            1.706375053909089,
            0.5047621455575189,
            -1.6661934509388395,
            -1.0337042757247657,
            -0.41568294934346726,
            1.4536155554562367,
            0.4079876871561164
        ]
    }
    usability_rotation = {
        "left":[np.pi, 0.0, np.pi/2],
        "right": [np.pi, 0.0, np.pi/2]
    }
    trackpad_for_wrist_rot = True
    trackpad_rot_rate = 0.2

    # Gripper
    gripper_keypoints = [
        [0.0, 0.0, 0.0, 0.5],
        [0.37, 0.37, 0.37, 1.0],
        [0.51, 0.51, 0.51, 1.0],
        [1.0, 1.0, 1.0, 0.5],
    ]
    init_gripper_state=0

class VictorTeleopSimProfile(VictorTeleopRealRobotProfile):
    # Gripper
    gripper_keypoints = [
        [0.0, 0.0, 0.0, 0.5],
        [0.33, 0.33, 0.33, 1.0],
        [0.38, 0.38, 0.38, 1.0],
        [1.0, 1.0, 1.0, 0.5],
    ]
    usability_rotation = {
        "left":[np.pi, 0.0, np.pi/2],
        "right": [np.pi, 0.0, np.pi/2],
        # "viewport":[np.pi/2, 0.0, np.pi/2],
        "viewport":[2.973773, -0.169581, 0.0]
    }
    init_gripper_state=0
    use_viewport_tracker = True

    # Viewport tracker settings for simulation
    viewport_position_sensitivity = 2.0
    viewport_orientation_sensitivity = 1.0

# Scan this entire file and register profile classes that are subclasses of VictorTeleopProfile
for name, obj in list(globals().items()):
    if isinstance(obj, type) and issubclass(obj, VictorTeleopProfile) and obj is not VictorTeleopProfile:
        teleop_profile_dict[name] = obj
