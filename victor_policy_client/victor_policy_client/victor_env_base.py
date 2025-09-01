# A runner implementation for the physical Victor robot that
# mimics a Gym environment APIs

from abc import ABC, abstractmethod
import threading
from time import perf_counter, sleep
from typing import Tuple, List, Dict, Optional

import numpy as np
import torch

from .victor_policy_client import VictorPolicyClient
from .data_utils import SmartDict, SmartQueue

class BaseVictorArmActionManager:
    """
    Action manager that handles 6-DOF tool pose actions from DFMMPolicy
    """
    def __init__(self, n_action_steps):
        """Boilerplate template"""
        self.n_action_steps = n_action_steps

    @abstractmethod
    def _queue_action(self, actions: List[np.ndarray]):
        """
        Internal method for queueing list of actions
        **overload**

        Args:
            actions (List[np.ndarray]): List of actions (arm_dim + gripper+dim) to queue
        """
        pass

    def add_actions(self, action_tensor):
        """
        Add (T, **) shaped tensor to internal

        Splits (T, **) shaped tensor to a list of (**) shaped tensor with T elements
        then queue individually
        """
        n_steps = min(self.n_action_steps, action_tensor.shape[0])
        actions = [
            action_tensor[j] for j in range(n_steps)
            if not np.any(np.isnan(action_tensor[j]))
        ]
        self._queue_action(actions)

    @abstractmethod
    def _get_next_action(self):
        """
        Internal method for getting next action from internal storage
        **overload for other impl**
        """
        pass

    @abstractmethod
    def act(self):
        """
        Abstract method for sending a single command
        **overload**
        """
        pass

class VictorArmActionManager(BaseVictorArmActionManager):
    """
    Action manager that handles 6-DOF tool pose actions from DFMMPolicy
    """
    def __init__(self,
        arm,
        control_mode:str = "joint",     # joint or ee
        gripper_dim=4,
        n_action_steps=1,
        **kwargs
    ):
        super().__init__(n_action_steps)
        self.arm = arm
        # Configure
        assert control_mode in ["joint", "ee"], "Invalid control mode"
        self.control_mode = control_mode
        self.arm_dim = 7   # for both ee and joint
        self.gripper_dim = gripper_dim
        self.n_action_steps = n_action_steps
        
        self.action_lock = threading.Lock()
        self.actions = []

    def _queue_action(self, actions: List[np.ndarray]):
        """
        Internal method for queueing list of actions

        Args:
            actions (List[np.ndarray]): List of actions (arm_dim + gripper+dim) to queue
        """
        with self.action_lock:
            while len(self.actions) > 0:
                self.actions.pop(0)  # Clear queue
            for action in actions:
                assert action.shape[-1] == self.arm_dim + self.gripper_dim
                self.actions.append(action)

    def _get_next_action(self):
        """
        Internal method for getting next action from internal storage
        **overload for other impl**
        """
        if len(self.actions) == 0:
            return None
        return self.actions.pop(0)

    # assumes [7 dim joint angles, 1 dim gripper]
    def act(self):
        assert self.arm is not None
        with self.action_lock:
            action = self._get_next_action()
        if action is None:
            return None
        if self.control_mode == "joint":
            self.arm.set_joint_cmd(np.array(action[:self.arm_dim]))
        else:
            self.arm.set_cartesian_cmd(np.array(action[:self.arm_dim]))
        self.arm.set_gripper_cmd(np.array(action[self.arm_dim:self.arm_dim+self.gripper_dim]))
        return action

class VictorArmActionChunkingManager(VictorArmActionManager):
    def __init__(self,
        arm,
        control_mode:str = "joint",     # joint or ee
        gripper_dim=4,
        n_action_steps=1,
        hz=10,
        **kwargs
    ):
        super().__init__(arm, control_mode, gripper_dim, n_action_steps)

        # Define a timer
        self.wait_period = 1.0 / hz
        self.start_time = None
        # Override actions to use timestamped actions
        self.actions = []  # List of (timestamp_idx, [actions]) tuples
    
    def _queue_action(self, actions: List[np.ndarray], inference_timestamp:Optional[float] = None):
        """
        Internal method for queueing list of actions

        Args:
            actions (List[np.ndarray]): List of actions (arm_dim + gripper+dim) to queue

        In this fancier version, it checks for timestamp, infers a list of timestamps
        """
        # Get the start of inference time
        infer_counter = inference_timestamp or perf_counter()
        if self.start_time is None:
            self.start_time = infer_counter

        # Get index from time elapse
        idx = int((infer_counter - self.start_time) / self.wait_period)
        action_ids = list(range(idx, idx + len(actions)))

        # Then insert new actions
        with self.action_lock:
            if len(self.actions) == 0:
                list_start_idx = idx
            else:
                list_start_idx = self.actions[0][0]     # Get the idx of first element

            for action_id, action in zip(action_ids, actions):
                assert action.shape[-1] == self.arm_dim + self.gripper_dim
                # Get the "precise" index of list that this element would be in
                idx_in_list = action_id - list_start_idx

                # If new index, add as a tuple, otherwise append to existing list
                if idx_in_list >= len(self.actions):
                    self.actions.append((action_id, [action]))
                else:
                    self.actions[idx_in_list][1].append(action)
    
    def _get_next_action(self):
        """
        Internal method for getting next action from internal storage
        **overload for other impl**
        """
        # Remove unused actions from the list up to current timestamp
        curr_timestamp = perf_counter()
        curr_idx = int((curr_timestamp - self.start_time) / self.wait_period)
        with self.action_lock:
            # Clear out time-out-ed actions
            while self.actions and self.actions[0][0] < curr_idx:
                self.actions.pop(0)
            # Check for empty
            if len(self.actions) == 0:
                return None
            # Stack and get mean
            all_actions_pred = self.actions[0][1]
            all_actions_np = np.stack(all_actions_pred, axis=0)
        return all_actions_np.mean(axis=0)


class VictorPhysicalEnvBase(ABC):
    """
    **Extend this class**

    Base class for a Physical environment for the Victor robot
    """

    action_manager_classes = {
        "VictorArmActionManager": VictorArmActionManager,
        "VictorArmActionChunkingManager": VictorArmActionChunkingManager
    }

    def __init__(self,
        client: VictorPolicyClient,
        side: str,
        device: torch.device,
        hz=10,
        his_len=5,
        n_action_steps=1,
        action_manager:str = "VictorArmActionManager",
        control_mode:str='joint'
    ):
        self.client = client
        self.side = side
        self.device = device
        self.hz = hz
        self.arm = self.client.right if self.side == 'right' else self.client.left

        # Setup observation
        self.setup_pub()
        self.setup_sub()

        # Setup action
        assert action_manager in self.action_manager_classes.keys(), \
            f"Unknown action manager type {action_manager}"
        action_mgr_class = self.action_manager_classes[action_manager]
        self.control_mode = control_mode
        self.action_manager: VictorArmActionManager = action_mgr_class(
            self.arm,
            control_mode,
            n_action_steps=n_action_steps,
            hz=hz,
        )

        # Setup observation
        self.his_len = his_len
        self.obs_cur = None     # current observation SmartDict

        # Daemon info
        self.wait_period = 1.0 / hz
        self._as_daemon = False
        self._daemon_running = False
        self._daemon_thread = None
        self._daemon_lock = threading.Lock()
        self._stop_event = threading.Event()

    def setup_sub(self, **kwargs):
        """
        Custom method to setup additional subscribers
        - Register your pub under `self.client`
        """
        return

    def setup_pub(self, **kwargs):
        """
        Custom method to setup additional publishers
        - Register your sub under `self.client`
        """
        return

    def reset(self) -> Tuple[SmartDict, Dict]:
        self.obs_data = SmartQueue(max_size=self.his_len, backend="torch")
        # Lazy get obs since there is no sense making this early.
        self.obs_cur, extras = self.get_new_obs()
        return self.obs_cur, extras

    @abstractmethod
    def _process_raw_obs_frame(self) -> Tuple[float, SmartDict]:
        """
        Internal method for getting observations from ROS, etc. 

        **Overload**
        Return (timestamp, frame of observation in SmartDict)
        """
        pass

    def get_new_obs(self) -> Tuple[Optional[SmartDict], Dict]:
        """
        Public interface for getting current observations for policy
        
        return (obs_cur in SmartDict (with history), extras)
        """
        obs_counter, obs_frame = self._process_raw_obs_frame()

        # Handle case where obs_frame is None (e.g., in replay mode at end)
        if obs_frame is None:
            return (None, {})

        # Update observation history
        for key in obs_frame.keys():
            self.obs_data.add(key, obs_frame[key])
        self.obs_cur = self.obs_data.apply(lambda x: torch.stack(x, 1))
        extras = {
            "timestamp": obs_counter,
            "obs_frame": obs_frame
        }
        return self.obs_cur, extras

    def get_cached_obs(self):
        """
        Get the most recently cached observations without triggering new processing.
        Useful for daemon mode where observations are continuously updated in background.
        """
        return self.obs_cur

    def is_daemon_running(self):
        """Check if the daemon is currently running"""
        return self._as_daemon and self._daemon_running

    def add_actions(self, action_tensor):
        """Add actions from policy

        Args:
            action_tensor (torch.Tensor): Action tensor to add
        """
        self.action_manager.add_actions(action_tensor)

    def step(self, wait_for_obs=True):
        """
        actions: numpy array of (T, D]) where T is smallest-first (first action is actions[0])
        """
        self.action_manager.act()
        if not wait_for_obs:
            return None
        sleep(self.wait_period)
        return self.get_new_obs()

    def start_daemon(self):
        """Start async timer action/observation loop """
        if self._as_daemon:
            print("Daemon already running")
            return

        with self._daemon_lock:
            self._daemon_running = True
            self._as_daemon = True
            self._stop_event.clear()
            self._daemon_thread = threading.Thread(target=self.async_runner, daemon=True)
            self._daemon_thread.start()
        print(f"Started daemon thread for {self.side} arm at {self.hz}Hz")

    def stop_daemon(self):
        """Stop async timer"""
        if not self._as_daemon:
            return
            
        with self._daemon_lock:
            self._daemon_running = False
            self._as_daemon = False
            self._stop_event.set()
            
        if self._daemon_thread and self._daemon_thread.is_alive():
            self._daemon_thread.join(timeout=2.0)
        print(f"Stopped daemon thread for {self.side} arm")

    def async_runner(self):
        """Daemon thread for running actions/getting observation asynchronously"""
        while not self._stop_event.is_set():
            loop_start = perf_counter()
            
            try:
                self.step()
            except Exception as e:
                print(f"Error in async runner: {e}")
                
            # Maintain timing
            loop_time = perf_counter() - loop_start
            sleep_time = max(0, self.wait_period - loop_time)
            if sleep_time > 0:
                self._stop_event.wait(sleep_time)