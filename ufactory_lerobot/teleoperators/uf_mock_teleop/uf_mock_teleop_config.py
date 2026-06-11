#!/usr/bin/env python

# Copyright 2025 UFACTORY Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Tuple
from dataclasses import dataclass

from lerobot.teleoperators import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("uf::mock_teleop")
@dataclass
class UFMockTeleopConfig(TeleoperatorConfig):
    robot_ip: str = "192.168.1.127"
    control_space: str = "joint"    # joint/cartesian
    mock_type: int = 1
    mock_x_range: Tuple[float, ...] = (180, 600)       # mm
    mock_y_range: Tuple[float, ...] = (-280, 50)       # mm
    mock_z_range: Tuple[float, ...] = (180, 180)       # mm
    mock_roll_range: Tuple[float, ...] = (180, 180)    # °
    mock_pitch_range: Tuple[float, ...] = (0, 0)       # °
    mock_yaw_range: Tuple[float, ...] = (-90, 90)      # °
    mock_tcp_speed: int = 200           # mm/s
    hover_offset: int = 200             # mm
    grasp_tcp_speed: int = 150          # mm/s

    place_tcp_speed: int = 100          # mm/s
    place_tcp_pose: Tuple[float, ...] = (383, 253, 320, 180, 0, 0)  # x,y,z in mm; roll,pitch,yaw in °
    initial_tcp_pose: Tuple[float, ...] = (450, 0, 520, 180, 0, 0, 0)  # x,y,z in mm; roll,pitch,yaw in °

    fps: int = 30  # Hz

    gripper_type: int = 1  # 0: no gripper, 1: xArm Gripper, 2: xArm Gripper G2
    gripper_freq: int = 50  # Hz
    gripper_open: int = 800
    gripper_close: int = 0
