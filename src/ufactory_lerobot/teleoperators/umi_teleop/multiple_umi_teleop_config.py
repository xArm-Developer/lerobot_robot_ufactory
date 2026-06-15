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
from .umi_teleop_config import UmiTeleopConfig


@TeleoperatorConfig.register_subclass("uf::multiple_umi_teleop")
@dataclass
class MultipleUmiTeleopConfig(TeleoperatorConfig):
    teleops: dict[str, UmiTeleopConfig]

    def __post_init__(self):
        self.id = 'multiple_umi_teleop' if self.id is None else self.id