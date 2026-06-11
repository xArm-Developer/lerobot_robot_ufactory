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

from lerobot.teleoperators import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("uf::spacemouse_teleop")
@dataclass
class SpaceMouseTeleopConfig(TeleoperatorConfig):
    # Port to connect to the arm
    max_value: int = 300
    deadzone: tuple = (0,0,0,0,0,0)
    use_gripper: bool = False
    frequency: int = 10 # hz
    max_pos_speed: int = 250 # mm/s
    # Others: Calibration angles, joint directions etc.
