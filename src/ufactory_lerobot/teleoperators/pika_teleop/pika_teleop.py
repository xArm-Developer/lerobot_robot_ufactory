#!/usr/bin/env python

import time
import math
from typing import Any
from threading import Thread, Event, Lock
from lerobot.utils.errors import DeviceNotConnectedError
from ufactory_lerobot.devices.pika import PikaDevice
from ufactory_lerobot.devices.umi.vive_tracker.transformations import Transformations
from ..base_teleop import UFBaseTeleop
from .pika_teleop_config import PikaTeleopConfig


class PikaTeleop(UFBaseTeleop, Thread):
    
    config_class = PikaTeleopConfig
    name = "Pika Teleop For xArm"

    def __init__(self, config: PikaTeleopConfig, prefix=''):
        
        super().__init__(config)
        Thread.__init__(self) # Do NOT REMOVE!
        self.stop_event = Event()
        self.config = config
        self._is_connected = False
        self._is_calibrated = True
        self._data_lock = Lock()
        self._teleop_enabled = False
        self._last_action = None
        self._need_initial = False
        self.prefix = '' if not prefix else f'{prefix}.'

        tracker_to_robot_eef = list(self.config.tracker_to_robot_eef[:3]) + list(map(math.radians, self.config.tracker_to_robot_eef[3:6]))
        self.tracker_to_robot_matrix = Transformations.xyzrpy_to_rotation_matrix(*tracker_to_robot_eef)
        robot_base_pose = list(self.config.robot_base_pose[:3]) + list(map(math.radians, self.config.robot_base_pose[3:6]))
        self.robot_base_matrix = Transformations.xyzrpy_to_rotation_matrix(*robot_base_pose)
        self.begin_tracker_robot_matrix = None
        self._last_robot_pose = Transformations.rotation_matrix_to_xyzrxryrz(self.robot_base_matrix)
        self._last_gripper_pos = 0.0

        self.pika_device = PikaDevice(1, pika_sense_port=self.config.port)
        self.pika_sense = self.pika_device.pika_sense

    @property
    def action_features(self) -> dict:
        if self.config.use_gripper:
            return {
                "dtype": "float32",
                "shape": (7,),
                "names": {"pose.x": 0, "pose.y": 1, "pose.z": 2, "pose.rx": 3, "pose.ry": 4, "pose.rz": 5, "gripper.pos": 6},
            }
        else:
            return {
                "dtype": "float32",
                "shape": (6,),
                "names": {"pose.x": 0, "pose.y": 1, "pose.z": 2, "pose.rx": 3, "pose.ry": 4, "pose.rz": 5},
            }

    @property
    def feedback_features(self) -> dict:
        if self.config.use_gripper:
            return {
                "dtype": "float32",
                "shape": (7,),
                "names": {"pose.x": 0, "pose.y": 1, "pose.z": 2, "pose.rx": 3, "pose.ry": 4, "pose.rz": 5, "gripper.pos": 6},
            }
        else:
            return {
                "dtype": "float32",
                "shape": (6,),
                "names": {"pose.x": 0, "pose.y": 1, "pose.z": 2, "pose.rx": 3, "pose.ry": 4, "pose.rz": 5},
            }

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return self._is_calibrated

    def calibrate(self) -> None:
        # CHECK!!
        pass

    def configure(self) -> None:
        pass

    def connect(self, calibrate: bool = False) -> None:
        self.start()

    def disconnect(self):
        if not self._is_connected:
            DeviceNotConnectedError(f"{self} is not connected.")

        self.stop_event.set()
        self._is_connected = False
        self.join()
    
    def set_teleop_enabled(self, enabled: bool, obs=None):
        with self._data_lock:
            if enabled:
                if obs is not None:
                    self._last_robot_pose = [obs[f"{self.prefix}pose.x"], obs[f"{self.prefix}pose.y"], obs[f"{self.prefix}pose.z"], obs[f"{self.prefix}pose.rx"], obs[f"{self.prefix}pose.ry"], obs[f"{self.prefix}pose.rz"]]
                    if self.config.use_gripper:
                        self._last_gripper_pos = obs[f"{self.prefix}gripper.pos"]
                    self.robot_base_matrix = Transformations.xyzrxryrz_to_rotation_matrix(*self._last_robot_pose)
                self.begin_tracker_robot_matrix = None
                self._last_action = None
                self._teleop_enabled = True
                print(f'[{self.prefix}PIKA] Teleoperation is start')
            else:
                obs = self._last_action
                self._last_robot_pose = [obs[f"{self.prefix}pose.x"], obs[f"{self.prefix}pose.y"], obs[f"{self.prefix}pose.z"], obs[f"{self.prefix}pose.rx"], obs[f"{self.prefix}pose.ry"], obs[f"{self.prefix}pose.rz"]]
                if self.config.use_gripper:
                    self._last_gripper_pos = obs[f"{self.prefix}gripper.pos"]
                self._teleop_enabled = False
                self._last_action = None
                print(f'[{self.prefix}PIKA] Teleoperation has paused')

    def run(self):
        self._is_connected = True
        init_state = self.pika_sense.get_command_state()
        curr_state = init_state
        sleep_time = 1 / self.config.frequency

        while not self.stop_event.is_set():
            time.sleep(sleep_time)

            state = self.pika_sense.get_command_state()
            if state != curr_state:
                curr_state = state
                if not self._teleop_enabled and curr_state != init_state:
                    self.set_teleop_enabled(True, self._last_action)
                    time.sleep(1)
                elif self._teleop_enabled and curr_state == init_state:
                    self.self.set_teleop_enabled(False)
                    continue

    # delta action
    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(
                "PikaTeleop is not connected. You need to run `connect()` before `get_action()`."
            )
        with self._data_lock:
            if self._last_action is None:
                self._last_action = {
                    f"{self.prefix}pose.x": self._last_robot_pose[0],
                    f"{self.prefix}pose.y": self._last_robot_pose[1],
                    f"{self.prefix}pose.z": self._last_robot_pose[2],
                    f"{self.prefix}pose.rx": self._last_robot_pose[3],
                    f"{self.prefix}pose.ry": self._last_robot_pose[4],
                    f"{self.prefix}pose.rz": self._last_robot_pose[5],
                }
                if self.config.use_gripper:
                    self._last_action.update({f"{self.prefix}gripper.pos": self._last_gripper_pos})
            if not self._teleop_enabled:
                return self._last_action

        pose = self.pika_sense.get_pose(self.pika_device.pika_tracker_device)
        if pose:
            x, y, z = pose.position[0] * 1000 * self.config.scale_xyz, pose.position[1] * 1000 * self.config.scale_xyz, pose.position[2] * 1000 * self.config.scale_xyz
            quaternion = pose.rotation
            tracker_robot_matrix = Transformations.tracker_pose_to_robot_matrix(x, y, z, quaternion, self.tracker_to_robot_matrix)
            if self.begin_tracker_robot_matrix is None:
                self.begin_tracker_robot_matrix = tracker_robot_matrix
            robot_target_pose = Transformations.tracker_robot_matrix_to_robot_pose(self.begin_tracker_robot_matrix, tracker_robot_matrix, self.robot_base_matrix, is_axis_angle=True)
            # print(['{:.6f}'.format(val) for val in robot_target_pose])
            self._last_action[f"{self.prefix}pose.x"] = robot_target_pose[0]
            self._last_action[f"{self.prefix}pose.y"] = robot_target_pose[1]
            self._last_action[f"{self.prefix}pose.z"] = robot_target_pose[2]
            self._last_action[f"{self.prefix}pose.rx"] = robot_target_pose[3]
            self._last_action[f"{self.prefix}pose.ry"] = robot_target_pose[4]
            self._last_action[f"{self.prefix}pose.rz"] = robot_target_pose[5]
        else:
            pass

        if self.config.use_gripper:
            distance  = min(max(self.pika_sense.get_gripper_distance(), 0), 100)
            if distance is not None:
                gripper_pos = (100 - distance) / (100 - 0)
            else:
                gripper_pos = 0.0
            self._last_action.update({f"{self.prefix}gripper.pos": gripper_pos})
        return self._last_action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError