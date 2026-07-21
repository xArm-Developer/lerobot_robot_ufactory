#!/usr/bin/env python

import math
import time
from typing import Any
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot_robot_ufactory.devices.umi.vive_tracker.transformations import Transformations
from lerobot_robot_ufactory.devices.umi.vive_tracker import ViveTracker
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

        # from lerobot_robot_ufactory.devices.umi.xvlib import XVLib
        # self.tracker = ViveTracker() if self.config.use_vive_tracker else None
        # self.xvlib = XVLib(self.config.serial_number, not self.config.use_vive_tracker, self.config.use_gripper)

        tracker_to_robot_eef = list(self.config.tracker_to_robot_eef[:3]) + list(map(math.radians, self.config.tracker_to_robot_eef[3:6]))
        self.tracker_to_robot_matrix = Transformations.xyzrpy_to_rotation_matrix(*tracker_to_robot_eef)
        robot_base_pose = list(self.config.robot_base_pose[:3]) + list(map(math.radians, self.config.robot_base_pose[3:6]))
        self.robot_base_matrix = Transformations.xyzrpy_to_rotation_matrix(*robot_base_pose)
        self.begin_tracker_robot_matrix = None
        self._last_robot_pose = Transformations.rotation_matrix_to_xyzrxryrz(self.robot_base_matrix)
        self._last_gripper_pos = 0.0
        self._last_timestamp = 0

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

    def wait_slam_ready(self, repeat_times=3, timeout=8.0, min_confidence=0.5, stable_frames=5) -> bool:
        def _wait_slam_ready():
            deadline = time.monotonic() + timeout
            ok_count = 0
            last_ts = None
            cnt = 0
            while time.monotonic() < deadline and self.is_connected:
                time.sleep(0.04)
                ret, pose = self.xvlib.xv_get_slam_data()
                if cnt % 25 == 0:
                    print(f'[{self.prefix}UMI{self.config.serial_number}] Waiting for SLAM ready ... confidence: {pose.confidence}, hostTimestamp: {pose.hostTimestamp}, edgeTimestampUs: {pose.edgeTimestampUs}')
                cnt += 1
                if ret == 0:
                    if pose.confidence < min_confidence or pose.hostTimestamp == last_ts:
                        ok_count = 0
                        last_ts = pose.hostTimestamp
                        continue
                    ok_count += 1
                    last_ts = pose.hostTimestamp
                    if ok_count >= stable_frames:
                        return True
            return False

        print(f'[{self.prefix}UMI{self.config.serial_number}] Waiting for SLAM ready ...')
        ready = _wait_slam_ready()
        if ready:
            print(f'[{self.prefix}UMI{self.config.serial_number}] ******* SLAM is ready! ******')
        else:
            for i in range(repeat_times):
                if not self.is_connected:
                    break
                print(f'[{self.prefix}UMI{self.config.serial_number}] ******* SLAM is not ready! ******, try {i+1}/{repeat_times}')
                self.xvlib.xv_slam_uninit()
                time.sleep(0.5)
                self.xvlib.xv_slam_init()
                time.sleep(0.5)
                ready = _wait_slam_ready()
                if ready:
                    print(f'[{self.prefix}UMI{self.config.serial_number}] ******* SLAM is ready! ******')
                    return
            print(f'[{self.prefix}UMI{self.config.serial_number}] ******* SLAM is not ready after {repeat_times} tries! ******')
            self._is_connected = False
            raise DeviceNotConnectedError(f'[{self.prefix}UMI{self.config.serial_number}] SLAM is not ready, please check the device and try again.')

    def connect(self, calibrate: bool = False) -> None:
        from lerobot_robot_ufactory.devices.umi.xvlib import XVLib
        self.tracker = ViveTracker() if self.config.use_vive_tracker else None
        self.xvlib = XVLib(self.config.serial_number, False, self.config.use_gripper)
        if not self.config.use_vive_tracker:
            time.sleep(1) # wait xvlib init
            self.xvlib.xv_slam_init()
            time.sleep(1) # wait slam init
        if self.config.use_gripper:
            self.xvlib.xv_clamp_stream_init()
        self._is_connected = True
        super().connect(calibrate)

        if not self.config.use_vive_tracker:
            self.wait_slam_ready()

    def disconnect(self):
        super().disconnect()
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
            print(f'[{self.prefix}UMI{self.config.serial_number}] Teleoperation is start')
        else:
            obs = self._last_action
            if obs:
                self._last_robot_pose = [obs[f"{self.prefix}pose.x"], obs[f"{self.prefix}pose.y"], obs[f"{self.prefix}pose.z"], obs[f"{self.prefix}pose.rx"], obs[f"{self.prefix}pose.ry"], obs[f"{self.prefix}pose.rz"]]
                if self.config.use_gripper:
                    self._last_gripper_pos = obs[f"{self.prefix}gripper.pos"]
            self._teleop_enabled = False
            self._last_action = None
            print(f'[{self.prefix}UMI{self.config.serial_number}] Teleoperation has paused')

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
                print(f'[{self.prefix}UMI{self.config.serial_number}] cant not get pose from vive tracker')
                ret, pose_data = self.xvlib.xv_get_slam_data()
                if ret != 0:
                    print(f'[{self.prefix}UMI{self.config.serial_number}] cant not get pose from xvlib, ret: {ret}')
                    return self._last_action
                elif pose_data.hostTimestamp == self._last_timestamp:
                    print(f'[{self.prefix}UMI{self.config.serial_number}] pose hostTimestamp is the same as last time, use last action')
                    return self._last_action
                self._last_timestamp = pose_data.hostTimestamp
        else:
            ret, pose_data = self.xvlib.xv_get_slam_data()
            if ret != 0:
                print(f'[{self.prefix}UMI{self.config.serial_number}] cant not get pose from xvlib, ret: {ret}')
                return self._last_action
            elif pose_data.confidence < 0.3:
                print(f'[{self.prefix}UMI{self.config.serial_number}] pose confidence is too low: {pose_data.confidence}, use last action')
                return self._last_action
            # elif pose_data.hostTimestamp == self._last_timestamp:
            #     print(f'[{self.prefix}UMI{self.config.serial_number}] pose hostTimestamp({pose_data.hostTimestamp} {self._last_timestamp}) is the same as last time, use last action')
            #     return self._last_action
            self._last_timestamp = pose_data.hostTimestamp
            # print(f'[{self.prefix}UMI{self.config.serial_number}] pose11: {pose_data.position.to_list(6)}, confidence: {pose_data.confidence}, hostTimestamp: {pose_data.hostTimestamp}, edgeTimestampUs: {pose_data.edgeTimestampUs}')
            # self.xvlib.xv_get_slam_pose(0)
            # print(f'[{self.prefix}UMI{self.config.serial_number}] pose22: {pose_data.position.to_list(6)}, confidence: {pose_data.confidence}, hostTimestamp: {pose_data.hostTimestamp}, edgeTimestampUs: {pose_data.edgeTimestampUs}')

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
