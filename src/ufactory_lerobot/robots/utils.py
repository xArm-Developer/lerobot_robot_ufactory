# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

from lerobot.robots.utils import make_robot_from_config as lerobot_make_robot_from_config
from lerobot.robots.config import RobotConfig
from lerobot.robots.robot import Robot


def make_robot_from_config(config: RobotConfig) -> Robot:
    if config.type == "uf::robot":
        from .uf_robot import UFRobot
        return UFRobot(config)
    elif config.type == "uf::multiple_robot":
        from .uf_robot import MultipleUFRobot
        return MultipleUFRobot(config)
    elif config.type == "uf::mock_robot":
        from .uf_mock_robot import UFMockRobot
        return UFMockRobot(config)
    elif config.type == "uf::multiple_mock_robot":
        from .uf_mock_robot import MultipleUFMockRobot
        return MultipleUFMockRobot(config)
    else:
        return lerobot_make_robot_from_config(config)
