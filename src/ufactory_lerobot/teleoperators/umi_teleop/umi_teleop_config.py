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


@TeleoperatorConfig.register_subclass("uf::umi_teleop")
@dataclass
class UmiTeleopConfig(TeleoperatorConfig):
    serial_number: str
    use_gripper: bool = True
    use_vive_tracker: bool = False
    vive_tracker_id: str = 'WM0'
    tracker_to_robot_eef: Tuple[float, ...] = (0, 0, 0, 0, 0, -90)  # [x, y, z, roll(°), pitch(°), yaw(°)]
    robot_base_pose: Tuple[float, ...] = (300, 0, 300, 180, -90, 0) # [x, y, z, roll(°), pitch(°), yaw(°)]

    def __post_init__(self):
        self.id = 'umi_teleop' if self.id is None else self.id
