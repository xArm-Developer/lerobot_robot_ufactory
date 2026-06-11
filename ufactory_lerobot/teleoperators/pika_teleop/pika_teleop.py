#!/usr/bin/env python

import time
import math
import numpy as np
from typing import Any
from threading import Thread, Event, Lock
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from xarm.wrapper import XArmAPI
from ufactory_lerobot.devices.pika import PikaDevice
from ufactory_lerobot.devices.umi.vive_tracker.transformations import Transformations
from ..base_teleop import UFBaseTeleop
from .pika_teleop_config import PikaTeleopConfig


class PikaTeleop(UFBaseTeleop, Thread):
    
    config_class = PikaTeleopConfig
    name = "Pika Teleop For xArm"

    def __init__(self, config: PikaTeleopConfig):
        
        super().__init__(config)
        Thread.__init__(self) # Do NOT REMOVE!
        self.stop_event = Event()
        self.config = config
        self._is_connected = False
        self._is_calibrated = True
        self._data_lock = Lock()
        self._ctrl_flag = False
        self._need_initial = False

        self.pika_device = PikaDevice(1, pika_sense_port=self.config.port)
        self.pika_sense = self.pika_device.pika_sense

        if self.config.robot_ip:
            self.arm = XArmAPI(self.config.robot_ip, is_radian=True)
        else:
            self.arm = None

        self._robot_target_pose = None
        self._gripper_target_pos = None

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
    
    def set_ctrl_status(self, status):
        if status:
            if not self._ctrl_flag:
                print('开始遥操作')
                self._ctrl_flag = True
                self._need_initial = True
        else:
            self._ctrl_flag = False
            self._need_initial = False
            print('停止遥操作')

    def run(self):
        self._is_connected = True
        init_state = self.pika_sense.get_command_state()
        curr_state = init_state

        last_gripper_distance = 0

        self._ctrl_flag = False # 是否开启遥操作
        self._need_initial = False

        sleep_time = 1 / self.config.frequency

        if self.arm:
            self.arm.set_linear_spd_limit_factor(2.0)

        pika_to_robot_eef = [0, 0, 0, math.pi, -math.pi / 2, 0] # rpy
        # pika_to_robot_eef = [0, 0, 0, math.pi, 0, 0]

        # pika坐标系到机械臂坐标系的变换关系对应的变换矩阵
        pika_to_robot_matrix = Transformations.xyzrpy_to_rotation_matrix(*pika_to_robot_eef)
        # 机械臂初始位置对应的变换矩阵
        # robot_base_matrix = Transformations.xyzrpy_to_rotation_matrix(*[0, 0, 190, -np.pi, -np.radians(41), 0])
        robot_base_matrix = Transformations.xyzrpy_to_rotation_matrix(*[300, 0, 365, np.pi, 0, 0])
        # pika初始位置转换到机械臂坐标系后对应的变换矩阵
        pika_begin_robot_matrix = None
        # pika目标位置转换到机械臂坐标系后对应的变换矩阵
        pika_end_robot_matrix = None

        scale_xyz = self.config.scale_xyz

        while not self.stop_event.is_set():
            time.sleep(sleep_time)

            if not self.arm and pika_begin_robot_matrix is None:
                pose = self.pika_sense.get_pose(self.pika_device.pika_tracker_device)
                if not pose:
                    continue
                x, y, z = pose.position[0] * 1000 * scale_xyz, pose.position[1] * 1000 * scale_xyz, pose.position[2] * 1000 * scale_xyz
                pika_begin_robot_matrix = Transformations.tracker_pose_to_robot_matrix(x, y, z, pose.rotation, pika_to_robot_matrix)
                robot_target_pose = Transformations.tracker_robot_matrix_to_robot_pose(pika_begin_robot_matrix, pika_begin_robot_matrix, robot_base_matrix, is_axis_angle=True)
                print('初始绑定, 当前Pika位置对应的机械臂目标位置: x={:.6f}, y={:.6f}, z={:.6f}, rx={:.6f}, ry={:.6f}, rz={:.6f}'.format(robot_target_pose[0], robot_target_pose[1], robot_target_pose[2], math.degrees(robot_target_pose[3]), math.degrees(robot_target_pose[4]), math.degrees(robot_target_pose[5])))
                continue

            state = self.pika_sense.get_command_state()
            if state != curr_state:
                curr_state = state
                if not self._ctrl_flag and curr_state != init_state:
                    self._ctrl_flag = True
                    self._need_initial = True
                    # self.robot_init()
                    print('开始遥操作')
                    time.sleep(1)
                elif self._ctrl_flag and curr_state == init_state:
                    self._ctrl_flag = False
                    print('停止遥操作')
                    continue
            
            if self._ctrl_flag and self.arm and (not self.arm.connected or self.arm.error_code != 0 or self.arm.state >= 4):
                print('机械臂原因, 遥操作自动停止')
                init_state = state
                curr_state = state
                self._ctrl_flag = False
                continue
            
            if not self._ctrl_flag:
                continue

            if self.config.use_gripper:
                distance  = min(max(self.pika_sense.get_gripper_distance(), 0), 100)

                if abs(last_gripper_distance - distance) > 2:
                    last_gripper_distance = distance
                    with self._data_lock:
                        self._gripper_target_pos = last_gripper_distance

            pose = self.pika_sense.get_pose(self.pika_device.pika_tracker_device)
            if not pose:
                continue
            x, y, z = pose.position[0] * 1000 * scale_xyz, pose.position[1] * 1000 * scale_xyz, pose.position[2] * 1000 * scale_xyz

            if not self.arm:
                # 只有PIKA设备, 没有机械臂
                pika_end_robot_matrix = Transformations.tracker_pose_to_robot_matrix(x, y, z, pose.rotation, pika_to_robot_matrix)
                robot_target_pose = Transformations.tracker_robot_matrix_to_robot_pose(pika_begin_robot_matrix, pika_end_robot_matrix, robot_base_matrix, is_axis_angle=True)

                if self._need_initial:
                    self._need_initial = False
                    print('[初始位置] x={:.6f}, y={:.6f}, z={:.6f}, rx={:.6f}, ry={:.6f}, rz={:.6f}'.format(robot_target_pose[0], robot_target_pose[1], robot_target_pose[2], math.degrees(robot_target_pose[3]), math.degrees(robot_target_pose[4]), math.degrees(robot_target_pose[5])))
            else:
                if self._need_initial:
                    self._need_initial = False
                    # _, robot_pos = self.arm.get_position()
                    _, robot_pos = self.arm.get_position(is_radian=True)
                    robot_base_pose = robot_pos
                    print('[初始] 机械臂位置: {}'.format(robot_pos))

                    # 机械臂初始位置对应的变换矩阵
                    robot_base_matrix = Transformations.xyzrpy_to_rotation_matrix(*robot_pos)

                    # pika初始位置转换到机械臂坐标系后对应的变换矩阵
                    pika_begin_robot_matrix = Transformations.tracker_pose_to_robot_matrix(x, y, z, pose.rotation, pika_to_robot_matrix)
                    pika_end_robot_matrix = pika_begin_robot_matrix
                else:
                    # pika目标位置转换到机械臂坐标系后对应的变换矩阵
                    pika_end_robot_matrix = Transformations.tracker_pose_to_robot_matrix(x, y, z, pose.rotation, pika_to_robot_matrix)

                robot_target_pose = Transformations.tracker_robot_matrix_to_robot_pose(pika_begin_robot_matrix, pika_end_robot_matrix, robot_base_matrix, is_axis_angle=True)

            with self._data_lock:
                self._robot_target_pose = robot_target_pose

    # delta action
    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(
                "PikaTeleop is not connected. You need to run `connect()` before `get_action()`."
            )

        with self._data_lock:
            if self._robot_target_pose is not None:
                robot_target_pose = self._robot_target_pose.copy()
            else:
                robot_target_pose = None
            if self._gripper_target_pos is not None:
                gripper_target_pos = (100 - self._gripper_target_pos) / (100 - 0)
            else:
                gripper_target_pos = 0.0

        if robot_target_pose is None:
            if self.arm:
                _, robot_target_pose = self.arm.get_position_aa(is_radian=True)
            else:
                # robot_target_pose = [0, 0, 190, -np.pi, -np.radians(41), 0]
                robot_target_pose = [300, 0, 365, np.pi, 0, 0]
        # print(self._robot_target_pose, robot_target_pose)

        # output is delta change of the robot pose
        action_dict = {
            "pose.x": robot_target_pose[0],
            "pose.y": robot_target_pose[1],
            "pose.z": robot_target_pose[2],
            "pose.rx": robot_target_pose[3],
            "pose.ry": robot_target_pose[4],
            "pose.rz": robot_target_pose[5],
        }

        if self.config.use_gripper:
            action_dict.update({"gripper.pos": gripper_target_pos})

        return action_dict

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError