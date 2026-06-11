#!/usr/bin/env python

import numpy as np
from dataclasses import dataclass
from typing import Tuple
from lerobot.teleoperators import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("uf::gello_teleop")
@dataclass
class GelloTeleopConfig(TeleoperatorConfig):
    # Port to connect to the gello dummy arm
    port: str = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAJZYC7-if00-port0"

    # Others: Calibration angles, joint directions etc
    joint_ids: Tuple[int, ...] = (1, 2, 3, 4, 5, 6, 7)
    joint_signs: Tuple[int, ...] = (1, 1, 1, 1, 1, 1, 1) # if follow the original open-sourced gello xarm7 setup
    start_joints: Tuple[float, ...] = (0, 0, 0, np.pi/2, 0, np.pi/2, 0)
    gripper_id: int = 8  # -1: no gripper
    torque_joint_ids: Tuple[int, ...] = None  # the joints will activate torque mode.
