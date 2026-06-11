#!/usr/bin/env python

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

        if self.config.use_gripper:
            self.config.init_clamp_stream = True
        else:
            self.config.init_clamp_stream = False

        self.tracker = ViveTracker() if self.config.use_vive_tracker else None
        self.xvlib = XVLib(self.config.serial_number, self.config.init_slam, self.config.init_clamp_stream, self.config.init_color_camera, self.config.init_fisheye_cameras)
        
        # tracker_to_robot_eef = [0, 0, 0, math.pi / 2, -math.pi / 2, 0]
        # tracker_to_robot_eef = [0, 0, 0, 0, 0, -math.pi/2]
        # tracker_to_robot_eef = [0, 0, 0, math.pi, math.pi, 0] # Test1
        # tracker_to_robot_eef = [0, 0, 0, math.pi, math.pi, -math.pi/2] # Dual left
        # tracker_to_robot_eef = [0, 0, 0, math.pi, math.pi, math.pi/2] # Dual right
        tracker_to_robot_eef = self.config.tracker_to_robot_eef
        self.tracker_to_robot_matrix = Transformations.xyzrpy_to_rotation_matrix(*tracker_to_robot_eef)
        # robot_base_pose = [300, 0, 300, 0, 0, 0]
        # robot_base_pose = [300, 0, 300, math.pi, -math.pi/2, 0]
        # robot_base_pose = [220, 0, 385, math.pi, 0, 0]
        # robot_base_pose = [250, 0, 150, math.pi, 0, 0] # Test 1
        # robot_base_pose = [250, 0, 150, math.pi, 0, math.pi/2] # Dual left
        # robot_base_pose = [250, 0, 150, math.pi, 0, math.pi/2] # Dual right
        robot_base_pose = self.config.robot_base_pose
        self.robot_base_matrix = Transformations.xyzrpy_to_rotation_matrix(*robot_base_pose)
        self.begin_tracker_robot_matrix = None

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
        self.xvlib.xv_init(self.config.serial_number, self.config.init_slam, self.config.init_clamp_stream, self.config.init_color_camera, self.config.init_fisheye_cameras)
        self._is_connected = True

    def disconnect(self):
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

    def set_ctrl_status(self, status):
        if status:
            self.begin_tracker_robot_matrix = None
        else:
            pass

    # delta action
    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(
                "UmiTeleop is not connected. You need to run `connect()` before `get_action()`."
            )

        if self.tracker is not None:
            pose_data = self.tracker.get_pose(self.config.vive_tracker_id)
            if pose_data is None:
                print('cant not get pose from vive tracker')
                _, pose_data = self.xvlib.xv_get_slam_data()
        else:
            _, pose_data = self.xvlib.xv_get_slam_data()
        position = pose_data.position.to_list(6)
        quaternion = pose_data.quaternion.to_list(6)
        # orientation = pose_data.orientation.to_list()

        # x, y, z = position[0] * 1000, position[1] * 1000, position[2] * 1000
        # roll, pitch, yaw = math.degrees(orientation[0]), math.degrees(orientation[1]), math.degrees(orientation[2])
        # print(f'[1] x={x:.1f}, y={y:.1f}, z={z:.1f}, roll={roll:.1f}, pitch={pitch:.1f}, yaw={yaw:.1f}')

        # x, y, z = position[2] * 1000, position[0] * 1000, position[1] * 1000
        # roll, pitch, yaw = orientation[0], orientation[1], orientation[2]
        # roll, pitch, yaw = math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
        # print(f'[1] x={x:.1f}, y={y:.1f}, z={z:.1f}, roll={roll:.1f}, pitch={pitch:.1f}, yaw={yaw:.1f}')

        # x, y, z = position[0] * 1000, position[1] * 1000, position[2] * 1000
        # R_A = Transformations.quaternion_to_rotation_matrix(quaternion)
        # roll, pitch, yaw = Transformations.rotation_matrix_to_rpy(R_A)
        # roll, pitch, yaw = math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
        # print(f'[2] x={x:.1f}, y={y:.1f}, z={z:.1f}, roll={roll:.1f}, pitch={pitch:.1f}, yaw={yaw:.1f}')
        # print('*' * 50)
        # roll, pitch, yaw = math.degrees(orientation[0]), math.degrees(orientation[1]), math.degrees(orientation[2])
        # print(f'[1] x={x:.1f}, y={y:.1f}, z={z:.1f}, roll={roll:.1f}, pitch={pitch:.1f}, yaw={yaw:.1f}')

        x, y, z = position[0] * 1000, position[1] * 1000, position[2] * 1000
        tracker_robot_matrix = Transformations.tracker_pose_to_robot_matrix(x, y, z, quaternion, self.tracker_to_robot_matrix)
        if self.begin_tracker_robot_matrix is None:
            self.begin_tracker_robot_matrix = tracker_robot_matrix

        robot_target_pose = Transformations.tracker_robot_matrix_to_robot_pose(self.begin_tracker_robot_matrix, tracker_robot_matrix, self.robot_base_matrix, is_axis_angle=True)
        x, y, z = robot_target_pose[0:3]
        orientation = robot_target_pose[3:6]
        # roll, pitch, yaw = list(map(math.degrees, orientation))
        # print(f'[{self.config.serial_number}] x={x:.3f}, y={y:.3f}, z={z:.3f}, rx={roll:.3f}, ry={pitch:.3f}, rz={yaw:.3f}')

        # R_prev = Transformations.rpy_to_rotation_matrix(math.pi, -math.pi / 2, 0)
        # R_delta = Transformations.rxryrz_to_matrix(robot_target_pose[3:6])
        # R_curr = R_prev @ R_delta
        # # # R_curr = R_prev.apply(R_delta)
        # orientation = Transformations.rotation_matrix_to_rxryrz(R_curr)
        
        # roll, pitch, yaw = math.degrees(orientation[0]), math.degrees(orientation[1]), math.degrees(orientation[2])
        # print(f'[2] x={x:.1f}, y={y:.1f}, z={z:.1f}, rx={roll:.1f}, ry={pitch:.1f}, rz={yaw:.1f}')
        # print('*' * 50)

        # output is delta change of the robot pose
        action_dict = {
            # "pose.x": z,
            # "pose.y": -y,
            # "pose.z": x,
            # "pose.rx": orientation[2],
            # "pose.ry": -orientation[1],
            # "pose.rz": orientation[0],
            f"{self.prefix}pose.x": x,
            f"{self.prefix}pose.y": y,
            f"{self.prefix}pose.z": z,
            f"{self.prefix}pose.rx": orientation[0],
            f"{self.prefix}pose.ry": orientation[1],
            f"{self.prefix}pose.rz": orientation[2],
        }

        if self.config.use_gripper:
            _, clamp_data = self.xvlib.xv_get_clamp_stream_data()
            gripper_pos = (87 - clamp_data.data) / (87 - 0)
            action_dict.update({f"{self.prefix}gripper.pos": gripper_pos})

        return action_dict

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError
