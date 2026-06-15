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

from dataclasses import dataclass
from typing import Tuple
from lerobot.teleoperators import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("uf::pika_teleop")
@dataclass
class PikaTeleopConfig(TeleoperatorConfig):
    # Port to connect to the pika
    port: str = None
    frequency: int = 100 # hz
    use_gripper: bool = True
    scale_xyz: float = 1.0
    tracker_to_robot_eef: Tuple[float, ...] = (0, 0, 0, 180, -90, 0)    # [x, y, z, roll(°), pitch(°), yaw(°)]
    robot_base_pose: Tuple[float, ...] = (400, 0, 400, 180, 0, 0)       # [x, y, z, roll(°), pitch(°), yaw(°)]

    def __post_init__(self):
        self.id = 'pika_teleop' if self.id is None else self.id
