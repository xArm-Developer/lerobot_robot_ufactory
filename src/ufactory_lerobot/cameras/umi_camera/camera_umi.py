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

"""
Provides the RealSenseCamera class for capturing frames from Intel RealSense cameras.
"""

import logging
import time
from typing import Any
import cv2  # type: ignore  # TODO: add type stubs for OpenCV
from lerobot.cameras.camera import Camera
from ufactory_lerobot.devices.umi.xvlib import XVLib
from .configuration_umi import UmiCameraConfig
from lerobot.cameras.configs import ColorMode
from lerobot.cameras.utils import get_cv2_rotation


logger = logging.getLogger(__name__)


class UmiCamera(Camera):
    def __init__(self, config: UmiCameraConfig):
        """
        Initializes the RealSenseCamera instance.

        Args:
            config: The configuration settings for the camera.
        """

        super().__init__(config)

        self.config = config
        self.serial_number = self.config.serial_number

        self.fps = config.fps if config.fps else 30
        self.width = config.width if config.width else 1280
        self.height = config.height if config.height else 1280
        self.color_mode = config.color_mode
        self.use_depth = config.use_depth
        self.warmup_s = config.warmup_s
        self.rotation: int | None = get_cv2_rotation(config.rotation)

        self.last_frame = None
        self.xvlib = XVLib(self.serial_number)
        self.xvlib.xv_color_camera_init()

    def __str__(self) -> str:
        return f"{self.__class__.__name__}({self.serial_number})"

    @property
    def is_connected(self) -> bool:
        return True

    @staticmethod
    def find_cameras() -> list[dict[str, Any]]:
        return []

    def connect(self, warmup: bool = True) -> None:
        self.xvlib.xv_color_camera_init()
        if warmup:
            start_time = time.monotonic()
            while time.monotonic() - start_time < self.warmup_s:
                time.sleep(0.1)

    def read(self, color_mode = None):
        ret, img_data = self.xvlib.xv_get_color_image_rgb_data()
        if ret <= 0:
            return None
        requested_color_mode = self.color_mode if color_mode is None else color_mode
        if requested_color_mode not in (ColorMode.RGB, ColorMode.BGR):
            raise ValueError(
                f"Invalid color mode '{requested_color_mode}'. Expected {ColorMode.RGB} or {ColorMode.BGR}."
            )
        if requested_color_mode == ColorMode.RGB:
            frame = img_data.frame(rgb=True)
        else:
            frame = img_data.frame(rgb=False)
        if self.rotation in [cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE, cv2.ROTATE_180]:
            frame = cv2.rotate(frame, self.rotation)
        return frame

    def async_read(self, timeout_ms: float = 200):
        frame = self.read()
        if frame is not None:
            self.last_frame = frame
        else:
            frame = self.last_frame
        return frame

    def disconnect(self) -> None:
        pass
