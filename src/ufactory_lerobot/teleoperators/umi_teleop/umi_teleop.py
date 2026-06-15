#!/usr/bin/env python

import math
from typing import Any
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from ufactory_lerobot.devices.umi.vive_tracker.transformations import Transformations
from ufactory_lerobot.devices.umi.vive_tracker import ViveTracker
from ufactory_lerobot.devices.umi.xvlib import XVLib
from ..base_teleop import UFBaseTeleop
from .umi_teleop_config import UmiTeleopConfig


class UmiTeleop(UFBaseTeleop):
    
    config_class = UmiTeleopConfig
    name = "UMI Teleop For xArm"

    def __init__(self, config: UmiTeleopConfig, prefix=''):        
        super().__init__(config)
        self.config = config
        self.prefix = '' if not prefix else f'{prefix}.'
        self._is_connected = False
        self._is_calibrated = True
        self._teleop_enabled = False
        self._last_action = None

        self.tracker = None
        self.xvlib = None

        # self.tracker = ViveTracker() if self.config.use_vive_tracker else None
        # self.xvlib = XVLib(self.config.serial_number, not self.config.use_vive_tracker, self.config.use_gripper)

        tracker_to_robot_eef = list(self.config.tracker_to_robot_eef[:3]) + list(map(math.radians, self.config.tracker_to_robot_eef[3:6]))
        self.tracker_to_robot_matrix = Transformations.xyzrpy_to_rotation_matrix(*tracker_to_robot_eef)
        robot_base_pose = list(self.config.robot_base_pose[:3]) + list(map(math.radians, self.config.robot_base_pose[3:6]))
        self.robot_base_matrix = Transformations.xyzrpy_to_rotation_matrix(*robot_base_pose)
        self.begin_tracker_robot_matrix = None
        self._last_robot_pose = Transformations.rotation_matrix_to_xyzrxryrz(self.robot_base_matrix)
        self._last_gripper_pos = 0.0

    @property
    def action_features(self) -> dict:
        if self.config.use_gripper:
            return {
                "dtype": "float32",
                "shape": (7,),
                "names": {f"{self.prefix}pose.x": 0, f"{self.prefix}pose.y": 1, f"{self.prefix}pose.z": 2, f"{self.prefix}pose.rx": 3, f"{self.prefix}pose.ry": 4, f"{self.prefix}pose.rz": 5, f"{self.prefix}gripper.pos": 6},
            }
        else:
            return {
                "dtype": "float32",
                "shape": (6,),
                "names": {f"{self.prefix}pose.x": 0, f"{self.prefix}pose.y": 1, f"{self.prefix}pose.z": 2, f"{self.prefix}pose.rx": 3, f"{self.prefix}pose.ry": 4, f"{self.prefix}pose.rz": 5},
            }

    @property
    def feedback_features(self) -> dict:
        if self.config.use_gripper:
            return {
                "dtype": "float32",
                "shape": (7,),
                "names": {f"{self.prefix}pose.x": 0, f"{self.prefix}pose.y": 1, f"{self.prefix}pose.z": 2, f"{self.prefix}pose.rx": 3, f"{self.prefix}pose.ry": 4, f"{self.prefix}pose.rz": 5, f"{self.prefix}gripper.pos": 6},
            }
        else:
            return {
                "dtype": "float32",
                "shape": (6,),
                "names": {f"{self.prefix}pose.x": 0, f"{self.prefix}pose.y": 1, f"{self.prefix}pose.z": 2, f"{self.prefix}pose.rx": 3, f"{self.prefix}pose.ry": 4, f"{self.prefix}pose.rz": 5},
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
        self.tracker = ViveTracker() if self.config.use_vive_tracker else None
        self.xvlib = XVLib(self.config.serial_number, not self.config.use_vive_tracker, self.config.use_gripper)
        self._is_connected = True

    def disconnect(self):
        if self.xvlib:
            self.xvlib.xv_uninit()
        self._is_connected = False
    
    @staticmethod
    def normalize_angle(angle_deg):
        """
        将角度归一化到 [-180, 180] 区间
        """
        while angle_deg > 180:
            angle_deg -= 360
        while angle_deg < -180:
            angle_deg += 360
        return angle_deg

    def set_teleop_enabled(self, enabled: bool, obs=None):
        if enabled:
            if obs is not None:
                self._last_robot_pose = [obs[f"{self.prefix}pose.x"], obs[f"{self.prefix}pose.y"], obs[f"{self.prefix}pose.z"], obs[f"{self.prefix}pose.rx"], obs[f"{self.prefix}pose.ry"], obs[f"{self.prefix}pose.rz"]]
                if self.config.use_gripper:
                    self._last_gripper_pos = obs[f"{self.prefix}gripper.pos"]
                self.robot_base_matrix = Transformations.xyzrxryrz_to_rotation_matrix(*self._last_robot_pose)
            self.begin_tracker_robot_matrix = None
            self._last_action = None
            self._teleop_enabled = True
            print(f'[{self.prefix}UMI] Teleoperation is start')
        else:
            obs = self._last_action
            if obs:
                self._last_robot_pose = [obs[f"{self.prefix}pose.x"], obs[f"{self.prefix}pose.y"], obs[f"{self.prefix}pose.z"], obs[f"{self.prefix}pose.rx"], obs[f"{self.prefix}pose.ry"], obs[f"{self.prefix}pose.rz"]]
                if self.config.use_gripper:
                    self._last_gripper_pos = obs[f"{self.prefix}gripper.pos"]
            self._teleop_enabled = False
            self._last_action = None
            print(f'[{self.prefix}UMI] Teleoperation has paused')

    # delta action
    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(
                "UmiTeleop is not connected. You need to run `connect()` before `get_action()`."
            )

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

        if self.tracker is not None:
            pose_data = self.tracker.get_pose(self.config.vive_tracker_id)
            if pose_data is None:
                print('cant not get pose from vive tracker')
                _, pose_data = self.xvlib.xv_get_slam_data()
        else:
            _, pose_data = self.xvlib.xv_get_slam_data()
        position = pose_data.position.to_list(6)
        quaternion = pose_data.quaternion.to_list(6)

        x, y, z = position[0] * 1000, position[1] * 1000, position[2] * 1000
        tracker_robot_matrix = Transformations.tracker_pose_to_robot_matrix(x, y, z, quaternion, self.tracker_to_robot_matrix)
        if self.begin_tracker_robot_matrix is None:
            self.begin_tracker_robot_matrix = tracker_robot_matrix

        robot_target_pose = Transformations.tracker_robot_matrix_to_robot_pose(self.begin_tracker_robot_matrix, tracker_robot_matrix, self.robot_base_matrix, is_axis_angle=True)
        self._last_action[f"{self.prefix}pose.x"] = robot_target_pose[0]
        self._last_action[f"{self.prefix}pose.y"] = robot_target_pose[1]
        self._last_action[f"{self.prefix}pose.z"] = robot_target_pose[2]
        self._last_action[f"{self.prefix}pose.rx"] = robot_target_pose[3]
        self._last_action[f"{self.prefix}pose.ry"] = robot_target_pose[4]
        self._last_action[f"{self.prefix}pose.rz"] = robot_target_pose[5]

        if self.config.use_gripper:
            _, clamp_data = self.xvlib.xv_get_clamp_stream_data()
            gripper_pos = (87 - clamp_data.data) / (87 - 0)
            self._last_action.update({f"{self.prefix}gripper.pos": gripper_pos})

        return self._last_action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError
