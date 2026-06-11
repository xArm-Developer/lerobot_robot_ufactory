#!/usr/bin/env python

import time
import socket
import random
from typing import Any
import threading
from xarm.wrapper import XArmAPI
from xarm.core.utils import convert
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from lerobot.teleoperators import Teleoperator
from .uf_mock_teleop_config import UFMockTeleopConfig

RECORD_NONE = 0
RECORD_POSE = RECORD_NO_GRIPPER = 1
RECORD_GRIPPER = 2
RECORD_POSE_GRIPPER = RECORD_POSE | RECORD_GRIPPER


class UFMockTeleop(Teleoperator, threading.Thread):
    
    config_class = UFMockTeleopConfig
    name = "Mock Teleop For xArm"

    def __init__(self, config: UFMockTeleopConfig):
        
        super().__init__(config)
        threading.Thread.__init__(self) # Do NOT REMOVE!
        self.config = config
        self._is_connected = False
        self._is_calibrated = True # CHECK!!
        self._is_joint_space = config.control_space == 'joint'
        self._action_datas = []

        self.arm = XArmAPI(config.robot_ip, do_not_open=True)
        self.rt_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        assert len(self.config.mock_x_range) >= 1, 'The length of the parameter mock_x_range cannot be less than 1.'
        assert len(self.config.mock_y_range) >= 1, 'The length of the parameter mock_y_range cannot be less than 1.'
        assert len(self.config.mock_z_range) >= 1, 'The length of the parameter mock_z_range cannot be less than 1.'
        assert len(self.config.mock_roll_range) >= 1, 'The length of the parameter mock_roll_range cannot be less than 1.'
        assert len(self.config.mock_pitch_range) >= 1, 'The length of the parameter mock_pitch_range cannot be less than 1.'
        assert len(self.config.mock_yaw_range) >= 1, 'The length of the parameter mock_yaw_range cannot be less than 1.'

        self.mock_x_range = self.config.mock_x_range if len(self.config.mock_x_range) > 1 else (self.config.mock_x_range[0], self.config.mock_x_range[0]) 
        self.mock_y_range = self.config.mock_y_range if len(self.config.mock_y_range) > 1 else (self.config.mock_y_range[0], self.config.mock_y_range[0]) 
        self.mock_z_range = self.config.mock_z_range if len(self.config.mock_z_range) > 1 else (self.config.mock_z_range[0], self.config.mock_z_range[0]) 
        self.mock_roll_range = self.config.mock_roll_range if len(self.config.mock_roll_range) > 1 else (self.config.mock_roll_range[0], self.config.mock_roll_range[0]) 
        self.mock_pitch_range = self.config.mock_pitch_range if len(self.config.mock_pitch_range) > 1 else (self.config.mock_pitch_range[0], self.config.mock_pitch_range[0]) 
        self.mock_yaw_range = self.config.mock_yaw_range if len(self.config.mock_yaw_range) > 1 else (self.config.mock_yaw_range[0], self.config.mock_yaw_range[0]) 

        self._last_mock_x = self.mock_x_range[0]
        self._last_mock_y = self.mock_y_range[0]
        self._last_mock_z = self.mock_z_range[0]
        self._last_mock_roll = self.mock_roll_range[0]
        self._last_mock_pitch = self.mock_pitch_range[0]
        self._last_mock_yaw = self.mock_yaw_range[0]

        self._update_lock = threading.Lock()
        self._action_inx = 0
        self._record_status = 0  # 0: not recording; 1: recording joint/tcp pose; 3: recording joint/tcp pose and gripper pos
        self._report_gripper = False

    def __set_record_status(self, status: int):
        self._record_status = status
    
    def __reset_action_data(self):
        self.__set_record_status(RECORD_NONE)
        self._action_datas.clear()

    def __init_robot(self):
        self.arm.connect(self.config.robot_ip)
        self.arm.motion_enable()
        self.arm.set_mode(0)
        self.arm.set_state(0)

        if self.config.initial_tcp_pose is not None:
            self.arm.set_position(*self.config.initial_tcp_pose, speed=self.config.mock_tcp_speed, wait=True)
        if self.config.gripper_type > 0:
            self.arm.set_gripper_enable(True)
            self.arm.set_gripper_speed(3000)
            self.arm.set_gripper_position(800, wait=True)

            if self.arm.arm.version_is_ge(2, 7, 100) and hasattr(self.arm, 'set_external_device_monitor_params'):
                self.arm.set_external_device_monitor_params(self.config.gripper_type, self.config.gripper_freq)
                self._report_gripper = True

        self.rt_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rt_sock.setblocking(True)
        self.rt_sock.settimeout(1)
        self.rt_sock.connect((self.config.robot_ip, 30000))
        self._is_connected = True

    def __mock(self, events=None) -> bool:
        mock_tcp_speed = self.config.mock_tcp_speed
        grasp_tcp_speed = self.config.grasp_tcp_speed
        initial_tcp_pose = self.config.initial_tcp_pose
        self.arm.set_mode(0)
        self.arm.set_state(0)
        self.arm.set_position(*initial_tcp_pose, speed=mock_tcp_speed)
        if self.config.gripper_type > 0:
            self.arm.set_gripper_position(800, wait=True)

        if abs(self._last_mock_x - self.mock_x_range[0]) >= abs(self._last_mock_x - self.mock_x_range[1]):
            mock_x_range = [self.mock_x_range[0], self._last_mock_x]
        else:
            mock_x_range = [self._last_mock_x, self.mock_x_range[1]]
        if abs(self._last_mock_y - self.mock_y_range[0]) >= abs(self._last_mock_y - self.mock_y_range[1]):
            mock_y_range = [self.mock_y_range[0], self._last_mock_y]
        else:
            mock_y_range = [self._last_mock_y, self.mock_y_range[1]]
        if abs(self._last_mock_z - self.mock_z_range[0]) >= abs(self._last_mock_z - self.mock_z_range[1]):
            mock_z_range = [self.mock_z_range[0], self._last_mock_z]
        else:
            mock_z_range = [self._last_mock_z, self.mock_z_range[1]]
        if abs(self._last_mock_roll - self.mock_roll_range[0]) >= abs(self._last_mock_roll - self.mock_roll_range[1]):
            mock_roll_range = [self.mock_roll_range[0], self._last_mock_roll]
        else:
            mock_roll_range = [self._last_mock_roll, self.mock_roll_range[1]]
        if abs(self._last_mock_pitch - self.mock_pitch_range[0]) >= abs(self._last_mock_pitch - self.mock_pitch_range[1]):
            mock_pitch_range = [self.mock_pitch_range[0], self._last_mock_pitch]
        else:
            mock_pitch_range = [self._last_mock_pitch, self.mock_pitch_range[1]]
        if abs(self._last_mock_yaw - self.mock_yaw_range[0]) >= abs(self._last_mock_yaw - self.mock_yaw_range[1]):
            mock_yaw_range = [self.mock_yaw_range[0], self._last_mock_yaw]
        else:
            mock_yaw_range = [self._last_mock_yaw, self.mock_yaw_range[1]]
        
        x = random.uniform(mock_x_range[0], mock_x_range[1])
        y = random.uniform(mock_y_range[0], mock_y_range[1])
        z = random.uniform(mock_z_range[0], mock_z_range[1])
        roll = random.uniform(mock_roll_range[0], mock_roll_range[1])
        pitch = random.uniform(mock_pitch_range[0], mock_pitch_range[1])
        yaw = random.uniform(mock_yaw_range[0], mock_yaw_range[1])

        # x = random.uniform(self.mock_x_range[0], self.mock_x_range[1])
        # y = random.uniform(self.mock_y_range[0], self.mock_y_range[1])
        # z = random.uniform(self.mock_z_range[0], self.mock_z_range[1])
        # roll = random.uniform(self.mock_roll_range[0], self.mock_roll_range[1])
        # pitch = random.uniform(self.mock_pitch_range[0], self.mock_pitch_range[1])
        # yaw = random.uniform(self.mock_yaw_range[0], self.mock_yaw_range[1])

        print(f'\n[MOCK] x: {x:.1f} mm, y: {y:.1f} mm, z: {z:.1f} mm, roll: {roll:.1f} °, pitch: {pitch:.1f} °, yaw: {yaw:.1f} °')
        self._last_mock_x = x
        self._last_mock_y = y
        self._last_mock_z = z
        self._last_mock_roll = roll
        self._last_mock_pitch = pitch
        self._last_mock_yaw = yaw

        code = self.arm.set_position(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, speed=mock_tcp_speed, wait=True)
        if code != 0:
            print(f'[MOCK ERROR] Failed to move to mock position, error code: {code}')
            return False
        print('[MOCK] Reached mock position.')
        if events and events['exit_early']:  # exit early
            return False
        print('*** Place the target in the correct gripping position, adjust the robotic arm height.')
        input('*** Enter to continue >>> ')
        if events and events['exit_early']:  # exit early
            return False

        self.arm.set_mode(0)
        self.arm.set_state(0)

        _, tcp_target_pose = self.arm.get_position(is_radian=False)
        # 回到抓取目标正上方
        code = self.arm.set_position(z=z + self.config.hover_offset, speed=mock_tcp_speed, wait=True)
        if code or (events and events['exit_early']):  # exit early
            return False
        _, tcp_hover_pose = self.arm.get_position(is_radian=False)

        # 回到初始位置
        code = self.arm.set_position(*initial_tcp_pose, speed=mock_tcp_speed, wait=True)
        if code or (events and events['exit_early']):  # exit early
            return False

        ##################### 模拟抓取 ############################

        self.__reset_action_data()
        self._action_inx = 0
        self.__set_record_status(RECORD_POSE_GRIPPER)
        time.sleep(1.0)

        case_type = self.config.mock_type if self.config.mock_type in [1, 2, 3] else 1
        x0, y0, yaw0 = tcp_hover_pose[0], tcp_hover_pose[1], tcp_hover_pose[5]
        dis = 100
        yaw_dis = 45
        spd = grasp_tcp_speed // 3 * 2
        x_range = [x0 - dis, x0 + dis]
        y_range = [y0 - dis, y0 + dis]
        yaw_range = [yaw0 - yaw_dis, yaw0 + yaw_dis]

        if self.config.mock_type == 0:
            # 按一定概率随机执行方案1/2/3
            val = val = random.random()
            case_type = 1 if val < 0.66 else 2 if val < 0.88 else 3
        
        cnt = 0
        if case_type == 1:
            # 方案1: 不做额外处理
            pass
        elif case_type == 2:
            # 方案2: 在目标点位上方固定高度一定范围内游走
            cnt = random.randint(3, 8)
            for i in range(cnt):
                x1 = min(max(self.mock_x_range[0], random.uniform(x_range[0], x_range[1])), self.mock_x_range[1])
                y1 = min(max(self.mock_y_range[0], random.uniform(y_range[0], y_range[1])), self.mock_y_range[1])
                yaw1 = min(max(self.mock_yaw_range[0], random.uniform(yaw_range[0], yaw_range[1])), self.mock_yaw_range[1])
                pose = [x1, y1, tcp_hover_pose[2], tcp_hover_pose[3], tcp_hover_pose[4], yaw1]
                self.arm.set_position(*pose, speed=grasp_tcp_speed if i == 0 else spd, wait=True if i == cnt - 1 else False)
                
                x_range[0] = x1 if x1 < x0 else x0 if x1 == x0 else x_range[0]
                x_range[1] = x1 if x1 > x0 else x0 if x1 == x0 else x_range[1]
                y_range[0] = y1 if y1 < y0 else y0 if y1 == y0 else y_range[0]
                y_range[1] = y1 if y1 > y0 else y0 if y1 == y0 else y_range[1]
                yaw_range[0] = yaw1 if yaw1 < yaw0 else yaw0 if yaw1 == yaw0 else yaw_range[0]
                yaw_range[1] = yaw1 if yaw1 > yaw0 else yaw0 if yaw1 == yaw0 else yaw_range[1]
        elif case_type == 3:
            # 方案3: 在目标点位上方随机高度一定范围内游走
            cnt = random.randint(2, 5)
            for i in range(cnt):
                x1 = min(max(self.mock_x_range[0], random.uniform(x_range[0], x_range[1])), self.mock_x_range[1])
                y1 = min(max(self.mock_y_range[0], random.uniform(y_range[0], y_range[1])), self.mock_y_range[1])
                yaw1 = min(max(self.mock_yaw_range[0], random.uniform(yaw_range[0], yaw_range[1])), self.mock_yaw_range[1])
                z1 = tcp_hover_pose[2] - (random.uniform(0, self.config.hover_offset - 50) if i != 0 else 0)
                pose = [x1, y1, z1, tcp_hover_pose[3], tcp_hover_pose[4], yaw]
                self.arm.set_position(*pose, speed=grasp_tcp_speed if i == 0 else spd, wait=True)
                if tcp_hover_pose[2] - z1 > 30 or i == cnt - 1:
                    pose = [x1, y1, tcp_hover_pose[2], tcp_hover_pose[3], tcp_hover_pose[4], yaw1]
                    self.arm.set_position(*pose, speed=grasp_tcp_speed if i == 0 else spd, wait=True)
                
                x_range[0] = x1 if x1 < x0 else x0 if x1 == x0 else x_range[0]
                x_range[1] = x1 if x1 > x0 else x0 if x1 == x0 else x_range[1]
                y_range[0] = y1 if y1 < y0 else y0 if y1 == y0 else y_range[0]
                y_range[1] = y1 if y1 > y0 else y0 if y1 == y0 else y_range[1]
                yaw_range[0] = yaw1 if yaw1 < yaw0 else yaw0 if yaw1 == yaw0 else yaw_range[0]
                yaw_range[1] = yaw1 if yaw1 > yaw0 else yaw0 if yaw1 == yaw0 else yaw_range[1]
        
        print(f'[MOCK] mock_type={self.config.mock_type}, case_type={case_type}, cnt={cnt}')

        # 去到抓取目标正上方
        code = self.arm.set_position(*tcp_hover_pose, speed=grasp_tcp_speed, wait=True)
        if code or (events and events['exit_early']):  # exit early
            self.__reset_action_data()
            return False
        time.sleep(0.5)

        # 下移到抓取目标位置
        code = self.arm.set_position(*tcp_target_pose, speed=grasp_tcp_speed, wait=True)
        if code or (events and events['exit_early']):  # exit early
            self.__reset_action_data()
            return False
        time.sleep(0.25)

        if self.config.gripper_type > 0:
            # 抓取
            self.arm.set_gripper_position(0, wait=True)
            time.sleep(0.25)
            self.__set_record_status(RECORD_NONE)
            # 记录抓取时的夹爪位置
            _, gripper_pos = self.arm.get_gripper_position()
            # 松开(不真正抓取)
            self.arm.set_gripper_position(800, wait=True)

        self.__set_record_status(RECORD_NO_GRIPPER)
        # 回到抓取目标正上方
        code = self.arm.set_position(*tcp_hover_pose, speed=grasp_tcp_speed, wait=True)
        if code or (events and events['exit_early']):  # exit early
            self.__reset_action_data()
            return False
        if self.config.gripper_type > 0:
            # 把机械爪恢复成抓取时位置
            self.arm.set_gripper_position(gripper_pos)
        time.sleep(0.25)

        # 去放置位置正上方
        place_tcp_hover_pose = self.config.place_tcp_pose.copy()
        place_tcp_hover_pose[2] += self.config.hover_offset
        code = self.arm.set_position(*place_tcp_hover_pose, speed=grasp_tcp_speed, wait=True)
        if code or (events and events['exit_early']):  # exit early
            self.__reset_action_data()
            return False
        time.sleep(0.25)

        # 下移到放置位置
        place_tcp_target_pose = self.config.place_tcp_pose
        code = self.arm.set_position(*place_tcp_target_pose, speed=grasp_tcp_speed, wait=True)
        if code or (events and events['exit_early']):  # exit early
            self.__reset_action_data()
            return False
        time.sleep(0.25)

        self.__set_record_status(RECORD_POSE_GRIPPER)
        # 放下
        if self.config.gripper_type > 0:
            self.arm.set_gripper_position(800, wait=True)
        time.sleep(0.25)
        # 回到放置位置正上方
        code = self.arm.set_position(*place_tcp_hover_pose, speed=grasp_tcp_speed, wait=True)
        if code or (events and events['exit_early']):  # exit early
            self.__reset_action_data()
            return False
        time.sleep(0.25)
        # 回到初始位置
        code = self.arm.set_position(*initial_tcp_pose, speed=grasp_tcp_speed, wait=True)
        if code or (events and events['exit_early']):  # exit early
            self.__reset_action_data()
            return False
        time.sleep(0.25)
        self.__set_record_status(RECORD_NONE)

        print(f'[MOCK] Recorded {len(self._action_datas)} steps of action data.')

        self.arm.set_mode(6)
        self.arm.set_state(0)

        return True
    
    @property
    def action_features(self) -> dict:
        if self._is_joint_space:
            # Add one more dof for gripper
            return { f"J{i+1}.pos": float for i in range(7) } | {"gripper.pos": float}
        else:
            if self.config.gripper_type > 0:
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
        if self._is_joint_space:
            # Add one more dof for gripper
            return { f"J{i+1}.pos": float for i in range(7) } | {"gripper.pos": float}
        else:
            if self.config.gripper_type > 0:
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

    def configure(self, events=None) -> None:
        return self.__mock(events=events)

    def connect(self, calibrate: bool = False) -> None:
        self.__init_robot()
        self.start()
    
    def disconnect(self):
        self._is_connected = False
        self.arm.set_state(4)
        self.arm.disconnect()
        self.join()

    def run(self):
        sleep_time_s = 1 / self.config.fps
        next_record_time = time.perf_counter() + sleep_time_s

        buffer = self.rt_sock.recv(4)
        while len(buffer) < 4:
            buffer += self.rt_sock.recv(4 - len(buffer))
        size = convert.bytes_to_u32(buffer[:4])
        print(f"UFACTORY Robot RT Report Thread starts!! size={size}")
        gripper_pos = 1.0
        while self.is_connected:
            buffer += self.rt_sock.recv(size - len(buffer))
            if len(buffer) < size:
                continue
            data = buffer[:size]
            buffer = buffer[size:]
            
            with self._update_lock:
                if self._record_status:
                    time_now = time.perf_counter()
                    if time_now - next_record_time >= -0.001:
                        if self._is_joint_space:
                            pose = convert.bytes_to_fp32s(data[116:144], 7) # joint angles
                        else:
                            pose = convert.bytes_to_fp32s(data[472:496], 6) # tcp pose

                        if self.config.gripper_type > 0:
                            if self._record_status & RECORD_GRIPPER:
                                if size >= 744 and self._report_gripper:
                                    external_device_info = convert.bytes_to_16s(data[738:744], 3)   # gripper: [pos, speed, current]
                                    grippos = min(external_device_info[0] * 10, self.config.gripper_open)
                                else:
                                    _, grippos = self.arm.get_gripper_position()
                                    grippos = min(grippos, self.config.gripper_open)
                                grippos_norm = (self.config.gripper_open - grippos) / (self.config.gripper_open - self.config.gripper_close)
                                pose.append(grippos_norm)  # add gripper pos
                                gripper_pos = grippos_norm
                            else:
                                pose.append(gripper_pos)
                        else:
                            if self._is_joint_space:
                                pose.append(gripper_pos)

                        self._action_datas.append(pose)
                        next_record_time = time_now + sleep_time_s

    # delta action
    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(
                "UFMockTeleop is not connected. You need to run `connect()` before `get_action()`."
            )
        
        if not self._action_datas:
            if self._is_joint_space:
                _, pose = self.arm.get_servo_angle(is_radian=True)
                if self.config.gripper_type > 0:
                    _, grippos = self.arm.get_gripper_position()
                    grippos = min(grippos, self.config.gripper_open)
                    grippos_norm = (self.config.gripper_open - grippos) / (self.config.gripper_open - self.config.gripper_close)
                    pose.append(grippos_norm)
                else:
                    pose.append(1.0)  # gripper pos
            else:
                _, pose = self.arm.get_position_aa(is_radian=True)
                if self.config.gripper_type > 0:
                    _, grippos = self.arm.get_gripper_position()
                    grippos = min(grippos, self.config.gripper_open)
                    grippos_norm = (self.config.gripper_open - grippos) / (self.config.gripper_open - self.config.gripper_close)
                    pose.append(grippos_norm)
        else:
            if self._action_inx >= len(self._action_datas):
                pose = self._action_datas[-1]
            else:
                pose = self._action_datas[self._action_inx]
                self._action_inx += 1
        action = {}
        if self._is_joint_space:
            for i in range(self.arm.axis):
                action.update({f"J{i+1}.pos": pose[i]})
            action.update({"gripper.pos": pose[7]})
        else:
            action.update({
                "pose.x": pose[0],
                "pose.y": pose[1],
                "pose.z": pose[2],
                "pose.rx": pose[3],
                "pose.ry": pose[4],
                "pose.rz": pose[5],
            })
            if self.config.gripper_type > 0:
                action.update({"gripper.pos": pose[6]})
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError
