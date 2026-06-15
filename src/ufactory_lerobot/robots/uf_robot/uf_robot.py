#!/usr/bin/env python

import time
import math
import logging
import struct
import numpy as np
from enum import IntEnum
from dataclasses import dataclass
from threading import Thread, Event, Lock
from lerobot.robots import Robot
from lerobot.cameras.utils import make_cameras_from_configs
from ufactory_lerobot.devices.pika import PikaDevice
from .uf_robot_config import UFRobotConfig
from xarm.wrapper import XArmAPI
from xarm.core.utils import convert

## Configurations:
INIT_SYNC_JOINT_VELOCITY_RAD = 0.2

CARTESIAN_OBS_KEYS = [
    "pose.x", "pose.y", "pose.z", "pose.rx", "pose.ry", "pose.rz",
    # un-comment if you need more features below:
    # "velo.x", "velo.y", "velo.z", "velo.rx", "velo.ry", "velo.rz",
]

CARTESIAN_ACTION_KEYS = [
    "pose.x", "pose.y", "pose.z", "pose.rx", "pose.ry", "pose.rz",
]

class GripperType(IntEnum):
    NoGripper = 0
    xArmGripper = 1
    xArmGripperG2 = 2
    BioGripperG2 = 3
    PikaGripper = 10
    RobotiqGripper = 11


@dataclass
class GripperParam:
    name: str
    open_pos: int
    close_pos: int
    speed: int = 0
    force: int = 0
    gripper_norm: float = 0

    def get_grippos(self, gripper_norm):
        pos = self.open_pos + gripper_norm * (self.close_pos - self.open_pos)
        min_pos, max_pos = min(self.open_pos, self.close_pos), max(self.open_pos, self.close_pos)
        return int(min(max(min_pos, pos), max_pos))

    def get_gripper_norm(self, grippos):
        if grippos is None:
            return self.gripper_norm
        self.gripper_norm = (self.open_pos - grippos) / (self.open_pos - self.close_pos)
        return self.gripper_norm


class UFRobot(Robot, Thread):

    config_class = UFRobotConfig
    name = "UFACTORY Robot"

    def __init__(self, config: UFRobotConfig, prefix=''):
        super().__init__(config)
        Thread.__init__(self)
        self.prefix = '' if not prefix else f"{prefix}."
        self.config = config
        self._dof = config.robot_dof 
        if self._dof == None or (not self._dof in (5,6,7)):
            raise ValueError(f"Please specify the correct DOF uf_robot!, got {self._dof}")
        
        self._control_space = self.config.control_space

        self.real_arm = None
        self.cameras = make_cameras_from_configs(config.cameras)

        self._is_connected = False
        self._is_calibrated =True

        self.logs = {}

        self._cmd_cnt = 0

        self._max_joint_velocity = math.radians(self.config.max_joint_velocity)
        self._max_linear_velocity = self.config.max_linear_velocity

        if self.config.start_tcp_pose and len(self.config.start_tcp_pose) >= 6:
            self._start_tcp_pose = list(self.config.start_tcp_pose[:3]) + list(map(math.radians, self.config.start_tcp_pose[3:6]))
        else:
            self._start_tcp_pose = None
        if self.config.start_joints:
            self._start_joints = list(map(math.radians, self.config.start_joints))
        else:
            self._start_joints = None

        self.report_stop_event = Event()
        self._rt_report_normal = False
        self._update_lock = Lock()
        self._use_rt_report = (self._control_space == "cartesian") # Cartesian observations must utilize rt_report
        self._cart_obs_has_vel = any('velo.' in key for key in CARTESIAN_OBS_KEYS)
        self._jnt_obs_has_vel = self.config.observe_joint_vel

        self._gripper_type = self.config.gripper_type
        if self._gripper_type == GripperType.xArmGripper:
            gripper_speed = 5000 if self.config.gripper_speed < 0 else min(max(50, self.config.gripper_speed), 5000)
            gripper_force = 50 if self.config.gripper_force < 0 else self.config.gripper_force # # not support
            self._gripper_param = GripperParam('xArmGripper', open_pos=800, close_pos=0, speed=gripper_speed, force=gripper_force)
        elif self._gripper_type == GripperType.xArmGripperG2:
            speed = 225 if self.config.gripper_speed < 0 else min(max(15, self.config.gripper_speed), 225)
            gripper_speed = int(((speed * 60) / 9.88235 + 140) / 0.4)
            gripper_force = 50 if self.config.gripper_force < 0 else min(max(1, self.config.gripper_force), 100)
            self._gripper_param = GripperParam('xArmGripperG2', open_pos=84, close_pos=0, speed=gripper_speed, force=gripper_force)
        elif self._gripper_type == GripperType.BioGripperG2:
            gripper_speed = 2000 if self.config.gripper_speed < 0 else min(max(500, self.config.gripper_speed), 4500)
            gripper_force = 100 if self.config.gripper_force < 0 else min(max(1, self.config.gripper_force), 100)
            self._gripper_param = GripperParam('BioGripperG2', open_pos=150, close_pos=71, speed=gripper_speed, force=gripper_force)
        elif self._gripper_type == GripperType.PikaGripper:
            self.pika_device = PikaDevice(2, pika_gripper_port=self.config.gripper_port)
            self.pika_gripper = self.pika_device.pika_gripper
            logger = logging.getLogger('pika.gripper')
            logger.setLevel(logging.WARNING)
            gripper_speed = 0 if self.config.gripper_speed < 0 else self.config.gripper_speed # not support
            gripper_force = 0 if self.config.gripper_force < 0 else self.config.gripper_force # not support
            self._gripper_param = GripperParam('PikaGripper', open_pos=100, close_pos=0, speed=gripper_speed, force=gripper_force)
        elif self._gripper_type == GripperType.RobotiqGripper:
            gripper_speed = 255 if self.config.gripper_speed < 0 else min(max(1, self.config.gripper_speed), 255)
            gripper_force = 255 if self.config.gripper_force < 0 else min(max(1, self.config.gripper_force), 255)
            self._gripper_param = GripperParam('RobotiqGripper', open_pos=0, close_pos=0xFF, speed=gripper_speed, force=gripper_force)
        else: # no gripper or not support
            self._gripper_type = 0
            self._gripper_param = GripperParam('NoGripper', open_pos=0, close_pos=0, speed=0, force=0)

    @property
    def _robot_state_features(self)-> dict:
        if self._control_space == "joint":
            state_features = {f"{self.prefix}J{motor}.pos": float for motor in range(1, self._dof+1)}
            if self._jnt_obs_has_vel:
                state_features.update({f"{self.prefix}J{motor}.vel": float for motor in range(1, self._dof+1)})
            if self._gripper_type > GripperType.NoGripper:
                state_features.update({f"{self.prefix}gripper.pos": float})
        elif self._control_space == "cartesian":
            state_features = {f"{self.prefix}{key}": float for key in CARTESIAN_OBS_KEYS}
            if self._gripper_type > GripperType.NoGripper:
                state_features.update({f"{self.prefix}gripper.pos": float})
        else:
            raise ValueError(f"Please check the given control space of uf_robot! got {self._control_space}")
        return state_features

    @property
    # CHECK!! channel first or last?
    def _cam_features(self) -> dict:
        cam_ft = {}
        for cam_key, cam in self.cameras.items():
            cam_ft[f"{self.prefix}{cam_key}"] = (cam.height, cam.width, 3)
        return cam_ft

    @property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._robot_state_features, **self._cam_features}

    @property
    def action_features(self)-> dict:
        if self._control_space == "joint":
            action_ft = {f"{self.prefix}J{motor}.pos": float for motor in range(1, self._dof+1)}
        elif self._control_space == "cartesian":
            action_ft = {f"{self.prefix}{key}": float for key in CARTESIAN_ACTION_KEYS}
        else:
            raise ValueError(f"Please check the given control space of uf_robot! got {self._control_space}")
        # Consider adding velocity configuration ??
        if self._gripper_type > GripperType.NoGripper:
            action_ft.update({f"{self.prefix}gripper.pos": float})
        return action_ft

    def connect(self, calibrate: bool = True) -> None:
        self.real_arm = XArmAPI(self.config.robot_ip)
        time.sleep(0.2)
        self._is_connected = self.real_arm.connected
        if not self._is_connected:
            print(f"UF Robot connection Failed, please check the hardware availability at ip: {self.config.robot_ip}")
            raise ConnectionError()

        if not self._dof == self.real_arm.axis:
            print(f"[ERROR: ] Real Robot DOF({self.real_arm.axis}) does not match configuration ({self._dof})!")
            self._is_connected = False
            raise ConnectionError()

        for cam in self.cameras.values():
            cam.connect()
            self._is_connected = self._is_connected and cam.is_connected

        if not self._is_connected:
            print("Could not connect to the cameras, check that all cameras are plugged-in.")
            raise ConnectionError()

        # if self._gripper_type == GripperType.PikaGripper:
        #     if not self.pika_gripper.connect():
        #         print('Could not connect to pika gripper.')
        #         raise ConnectionError()

        self.configure()
        if calibrate:  
            self.calibrate()

        self.real_arm.set_linear_spd_limit_factor(2.0)

        self._is_connected = True

    def configure(self) -> None:
        self.real_arm.motion_enable()
        self.real_arm.clean_error()
        self.real_arm.set_mode(0)  # set to idle mode
        self.real_arm.set_state(0)  # set to start state
        time.sleep(0.5)

        _, err_warn = self.real_arm.get_err_warn_code()
        if err_warn[0] != 0:
            raise RuntimeError(f"Failed to set correct state to UF robot! Controller Error code: {err_warn[0]} !")

        if self._gripper_type > GripperType.NoGripper:
            self.real_arm._arm._baud_checkset = True
            if self._gripper_type == GripperType.xArmGripper:
                self.real_arm.set_gripper_enable(True)
                self.real_arm.set_gripper_mode(0)
                self.real_arm.set_gripper_speed(self._gripper_param.speed)
                self.real_arm.set_gripper_position(self._gripper_param.open_pos)
            elif self._gripper_type == GripperType.xArmGripperG2:
                self.real_arm.set_gripper_enable(True)
                self.real_arm.set_gripper_mode(0)
                self.real_arm.set_gripper_g2_position(self._gripper_param.open_pos)
            elif self._gripper_type == GripperType.BioGripperG2:
                _, mode = self.real_arm.get_bio_gripper_control_mode()
                if mode != 1:
                    self.real_arm.set_bio_gripper_control_mode(1)
                self.real_arm.set_bio_gripper_enable(True)
                self.real_arm.open_bio_gripper()
            elif self._gripper_type == GripperType.PikaGripper:
                self.pika_gripper.enable()
                time.sleep(0.5)
                self.pika_gripper.set_gripper_distance(self._gripper_param.open_pos)
            elif self._gripper_type == GripperType.RobotiqGripper:
                self.real_arm.robotiq_reset()
                self.real_arm.robotiq_set_activate(wait=True)
                self.real_arm.robotiq_set_position(self._gripper_param.open_pos, wait=True)
            self._gripper_param.grippos = self._gripper_param.open_pos
            self._gripper_param.gripper_norm = self._gripper_param.open_pos
            self.real_arm._arm._baud_checkset = False
            _, err_warn = self.real_arm.get_err_warn_code()
            if err_warn[0] != 0:
                raise RuntimeError(f"Failed to set correct state to Gripper! Controller Error code: {err_warn[0]} !")
        
        if self._start_joints is not None:
            self.real_arm.set_servo_angle(angle=self._start_joints, is_radian=True, wait=True)
        if self._start_tcp_pose is not None:
            self.real_arm.set_position(*self._start_tcp_pose, speed=100, is_radian=True, wait=True)
            _, self._start_joints = self.real_arm.get_servo_angle(is_radian=True)
            self._start_tcp_pose = None

        if self._control_space == "joint":
            self.real_arm.set_mode(6)
        elif self._control_space == "cartesian":
            self.real_arm.set_mode(7)
        else:
            raise ValueError(f"Please check the given control space of uf_robot! got {self._control_space}")

        self.real_arm.set_state(0)

        _, err_warn = self.real_arm.get_err_warn_code()
        if err_warn[0] != 0:
            raise RuntimeError(f"Failed to set correct state to UF robot! Controller Error code: {err_warn[0]} !")

        if self._use_rt_report and not self._rt_report_normal:
            self.start()
        time.sleep(0.2)

    def calibrate(self) -> None:
        self._is_calibrated = True
        pass # CHECK! currently No-op

    def get_observation(self) -> dict[str, np.ndarray]:
        obs_dict = {}

        # Read Stretch state
        before_read_t = time.perf_counter()
        if self._control_space == "joint":
            code, states = self.real_arm.get_joint_states(is_radian=True, num=3)
            pos_list = states[0].copy()
            obs_dict = {f"{self.prefix}J{k+1}.pos": pos_list[k] for k in range(self._dof)}
            if self._jnt_obs_has_vel:
                vel_list = states[1].copy()
                obs_dict.update({f"{self.prefix}J{k+1}.vel": vel_list[k] for k in range(self._dof)})
        elif self._control_space == "cartesian":
            if not self._rt_report_normal:
                raise ConnectionError("RT Report for target robot NOT READY! ")

            with self._update_lock:
                pos_list = self.rt_actual_tcp_pose.copy()
                vel_list = self.rt_actual_tcp_speed.copy()
                # pos_cmd_list = self.rt_cmd_tcp_pose.copy()
                # vel_cmd_list = self.rt_cmd_tcp_vel.copy()
                # jpos_fbk_list = self.rt_actual_joint_pos.copy()
                # jvel_fbk_list = self.rt_actual_joint_speed.copy()

            obs_dict = {f"{self.prefix}pose.x": pos_list[0], f"{self.prefix}pose.y": pos_list[1], f"{self.prefix}pose.z": pos_list[2], f"{self.prefix}pose.rx": pos_list[3], f"{self.prefix}pose.ry": pos_list[4], f"{self.prefix}pose.rz": pos_list[5]}
            if self._cart_obs_has_vel:
                obs_dict.update({f"{self.prefix}velo.x": vel_list[0], f"{self.prefix}velo.y": vel_list[1], f"{self.prefix}velo.z": vel_list[2], f"{self.prefix}velo.rx": vel_list[3], f"{self.prefix}velo.ry": vel_list[4], f"{self.prefix}velo.rz": vel_list[5]})
        else:
            ValueError(f"Please check the given control space of uf_robot! got {self._control_space}")
        
        if self._gripper_type > GripperType.NoGripper:
            if self._gripper_type == GripperType.xArmGripper:
                code, grippos = self.real_arm.get_gripper_position()
                grippos_norm = self._gripper_param.get_gripper_norm(grippos)
            elif self._gripper_type == GripperType.xArmGripperG2:
                code, grippos = self.real_arm.get_gripper_g2_position()
                grippos_norm = self._gripper_param.get_gripper_norm(grippos)
            elif self._gripper_type == GripperType.BioGripperG2:
                code, grippos = self.real_arm.get_bio_gripper_g2_position()
                grippos_norm = self._gripper_param.get_gripper_norm(grippos)
            elif self._gripper_type == GripperType.PikaGripper:
                grippos = self.pika_gripper.get_gripper_distance()
                grippos_norm = self._gripper_param.get_gripper_norm(grippos)
            elif self._gripper_type == GripperType.RobotiqGripper:
                self.real_arm.robotiq_get_status(number_of_registers=3)
                grippos = self.real_arm.robotiq_status['gPO']  # 0..255
                grippos_norm = self._gripper_param.get_gripper_norm(grippos) # 0=open, 1=closed
            self.logs["read_pos_dt_s"] = time.perf_counter() - before_read_t
            obs_dict[f"{self.prefix}gripper.pos"] = grippos_norm

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            before_camread_t = time.perf_counter()
            obs_dict[f"{self.prefix}{cam_key}"] = cam.async_read()
            self.logs[f"async_read_camera_{cam_key}_dt_s"] = time.perf_counter() - before_camread_t

        return obs_dict

    def send_action(self, action: dict) -> np.ndarray:
        if not self._is_connected:
            raise ConnectionError()
        if self.real_arm.error_code != 0:
            return action
        if self.config.no_action:
            return action

        before_write_t = time.perf_counter()
        if self._control_space == "joint":
            # first sync with gello or other control device SLOWLY!
            jnt_spd = INIT_SYNC_JOINT_VELOCITY_RAD if self._cmd_cnt < 20 else self._max_joint_velocity
            wait_ = True if self._cmd_cnt == 0 else False

            cmd_list = [0]*(self._dof)
            for i in range(self._dof):
                cmd_list[i] = action[f"{self.prefix}J{i+1}.pos"]

            # TODO: make mode 6 compatible with wait=True
            if wait_== False and self.real_arm.mode != 6:
                self.real_arm.set_mode(6)
                self.real_arm.set_state(0)
                time.sleep(0.1)
            elif wait_ and self.real_arm.mode != 0:
                self.real_arm.set_mode(0)
                self.real_arm.set_state(0)
                time.sleep(0.1)

            self.real_arm.set_servo_angle(angle=cmd_list[:self._dof], speed=jnt_spd, is_radian=True, wait=wait_)
        elif self._control_space == "cartesian": # unit: mm?
            lin_spd = self._max_linear_velocity

            if not self._rt_report_normal:
                raise ConnectionError("RT Report for target robot NOT READY! ")
            cmd_list = [action[f"{self.prefix}pose.x"], action[f"{self.prefix}pose.y"], action[f"{self.prefix}pose.z"], action[f"{self.prefix}pose.rx"], action[f"{self.prefix}pose.ry"], action[f"{self.prefix}pose.rz"]]
            self.real_arm.set_position_aa(axis_angle_pose=cmd_list, speed=lin_spd, is_radian=True, wait=False)
            # self.real_arm.set_position(*cmd_list, radius=0, speed=lin_spd, is_radian=True, wait=False)

        if self._cmd_cnt < 99999:
            self._cmd_cnt += 1 # CHECK!! possibility of overflow?
        if self._gripper_type > GripperType.NoGripper:
            gripper_norm = action[f"{self.prefix}gripper.pos"]
            if self._gripper_type == GripperType.xArmGripper:
                grippos = self._gripper_param.get_grippos(gripper_norm)
                modbus_datas = [0x08, 0x10, 0x07, 0x00, 0x00, 0x02, 0x04]
                modbus_datas.extend(list(struct.pack('>i', grippos)))
                self.real_arm.getset_tgpio_modbus_data(modbus_datas)
                # self.real_arm.set_gripper_position(grippos, wait=False, wait_motion=False) # CHECK! the command unit
            elif self._gripper_type == GripperType.xArmGripperG2:
                grippos = self._gripper_param.get_grippos(gripper_norm)
                grippos = int((math.degrees(math.asin((grippos - 16) / 110)) + 8.33) * 18.28)
                modbus_datas = [0x08, 0x10, 0x0C, 0x00, 0x00, 0x05, 0x0A, 0x00, 0x01]
                modbus_datas.extend(list(struct.pack('>h', self._gripper_param.speed)))
                modbus_datas.extend(list(struct.pack('>h', self._gripper_param.force)))
                modbus_datas.extend(list(struct.pack('>i', grippos)))
                self.real_arm.getset_tgpio_modbus_data(modbus_datas)
            elif self._gripper_type == GripperType.BioGripperG2:
                grippos = self._gripper_param.get_grippos(gripper_norm)
                grippos = int(grippos * 3.7342 - 265.13)
                modbus_datas = [0x08, 0x10, 0x0C, 0x00, 0x00, 0x05, 0x0A, 0x00, 0x01]
                modbus_datas.extend(list(struct.pack('>h', self._gripper_param.speed)))
                modbus_datas.extend(list(struct.pack('>h', self._gripper_param.force)))
                modbus_datas.extend(list(struct.pack('>i', grippos)))
                self.real_arm.getset_tgpio_modbus_data(modbus_datas)
            elif self._gripper_type == GripperType.PikaGripper:
                grippos = self._gripper_param.get_grippos(gripper_norm)
                self.pika_gripper.set_gripper_distance(grippos)
            elif self._gripper_type == GripperType.RobotiqGripper:
                grippos = self._gripper_param.get_grippos(gripper_norm)
                modbus_datas = [0x09, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06, 0x09, 0x00, 0x00, grippos, self._gripper_param.speed, self._gripper_param.force]
                self.real_arm.getset_tgpio_modbus_data(modbus_datas)
                # self.real_arm.robotiq_set_position(
                #     grippos, speed=self._gripper_param.speed, force=self._gripper_param.force,
                #     wait=False, wait_motion=False,
                # )

        self.logs["write_pos_dt_s"] = time.perf_counter() - before_write_t
        return action

    def print_logs(self) -> None:
        pass

    def disconnect(self) -> None:
        self.real_arm.set_state(4) # stop
        self.real_arm.set_mode(0)
        if self._use_rt_report:
            self.report_stop_event.set()
            self.join()
        self.real_arm.disconnect()
        # CHECK!! how about gripper? 

        for cam in self.cameras.values():
            cam.disconnect()

        self._is_connected = False

    def is_calibrated(self) -> bool:
        """Whether the robot is currently calibrated or not. Should be always `True` if not applicable"""
        return self._is_calibrated

    def is_connected(self) -> bool:
        """Whether the robot is currently calibrated or not. Should be always `True` if not applicable"""
        return self._is_connected

    def run(self):
        import socket
        
        robot_port = 30000 # DO NOT CHANGE
        # create socket connection
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setblocking(True)
        sock.settimeout(1)
        sock.connect((self.config.robot_ip, robot_port))

        buffer = sock.recv(4)
        print(buffer)
        while len(buffer) < 4:
            buffer += sock.recv(4 - len(buffer))
        size = convert.bytes_to_u32(buffer[:4])
        print(f"UFACTORY Robot ({self.config.robot_ip}) RT Report Thread starts!! =======")
        while not self.report_stop_event.is_set():
            buffer += sock.recv(size - len(buffer))
            if len(buffer) < size:
                continue
            data = buffer[:size]
            buffer = buffer[size:]
            with self._update_lock:
                self.rt_actual_joint_pos = convert.bytes_to_fp32s(data[116:144], 7)
                self.rt_actual_joint_speed = convert.bytes_to_fp32s(data[144:172], 7)
                self.rt_cmd_tcp_pose = convert.bytes_to_fp32s(data[424:448], 6)
                self.rt_cmd_tcp_vel = convert.bytes_to_fp32s(data[448:472], 6)
                self.rt_actual_tcp_pose = convert.bytes_to_fp32s(data[472:496], 6)
                self.rt_actual_tcp_speed = convert.bytes_to_fp32s(data[496:520], 6)
            self._rt_report_normal = True

        self._rt_report_normal = False
        print(f"UFACTORY Robot ({self.config.robot_ip}) RT Report Thread Exit!! =======")
