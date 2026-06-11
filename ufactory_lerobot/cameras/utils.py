#!/usr/bin/env python

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

from typing import cast
from lerobot.utils.import_utils import make_device_from_device_class
from lerobot.cameras.utils import make_cameras_from_configs as lerobot_make_cameras_from_configs
from lerobot.cameras.camera import Camera
from lerobot.cameras.configs import CameraConfig


def make_cameras_from_configs(camera_configs: dict[str, CameraConfig]) -> dict[str, Camera]:
    lerobot_camera_configs = {key: cfg for key, cfg in camera_configs.items() if not cfg.type.startswith("uf::")}
    uf_camera_configs = {key: cfg for key, cfg in camera_configs.items() if cfg.type.startswith("uf::")}
    cameras = lerobot_make_cameras_from_configs(lerobot_camera_configs)
    
    for key, cfg in uf_camera_configs.items():
        if cfg.type == "uf::umi_camera":
            from .umi_camera import UmiCamera
            cameras[key] = UmiCamera(cfg)
        else:
            try:
                cameras[key] = cast(Camera, make_device_from_device_class(cfg))
            except Exception as e:
                raise ValueError(f"Error creating camera with config {cfg}: {e}") from e
    return cameras
