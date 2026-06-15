#!/usr/bin/env python

import time
import numpy as np
from typing import Any
from threading import Thread, Event
from spnav import spnav_open, spnav_poll_event, spnav_close, SpnavMotionEvent, SpnavButtonEvent
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from collections import defaultdict

from lerobot.teleoperators import Teleoperator
from .space_mouse_config import SpaceMouseTeleopConfig

class SpaceMouseTeleop(Teleoperator, Thread):
    
    config_class = SpaceMouseTeleopConfig
    name = "Space Mouse Teleop For xArm"

    def __init__(self, config: SpaceMouseTeleopConfig):
        
        super().__init__(config)
        Thread.__init__(self) # Do NOT REMOVE!
        self.stop_event = Event()
        self.config = config
        self.max_value = config.max_value
        self.frequency = config.frequency
        self.max_pos_speed = config.max_pos_speed
        deadzone = config.deadzone
        self._is_connected = False
        self.dtype = np.float32 # CHECK! make it configurable ???

        if np.issubdtype(type(deadzone), np.number):
            self.deadzone = np.full(6, fill_value=deadzone, dtype=self.dtype)
        else:
            self.deadzone = np.array(deadzone, dtype=self.dtype)
        assert (self.deadzone >= 0).all()

        self.motion_event = SpnavMotionEvent([0,0,0], [0,0,0], 0)
        self.button_state = defaultdict(lambda: False)
        self.tx_zup_spnav = np.array([
            [0,0,-1],
            [1,0,0],
            [0,1,0]
        ], dtype=np.float32)

    @property
    def action_features(self) -> dict:
        if self.config.use_gripper:
            return {
                "dtype": "float32",
                "shape": (4,),
                "names": {"pose.dx": 0, "pose.dy": 1, "pose.dz": 2, "gripper.pos": 3},
            }
        else:
            return {
                "dtype": "float32",
                "shape": (3,),
                "names": {"pose.dx": 0, "pose.dy": 1, "pose.dz": 2},
            }

    @property
    def feedback_features(self) -> dict:
        if self.config.use_gripper:
            return {
                "dtype": "float32",
                "shape": (4,),
                "names": {"pose.dx": 0, "pose.dy": 1, "pose.dz": 2, "gripper.pos": 3},
            }
        else:
            return {
                "dtype": "float32",
                "shape": (3,),
                "names": {"pose.dx": 0, "pose.dy": 1, "pose.dz": 2},
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

    def configure(self) -> None:
        pass

    def connect(self, calibrate: bool = False) -> None:
        self.start()

    def disconnect(self):
        if not self._is_connected:
            DeviceNotConnectedError(f"{self} is not connected.")

        self.stop_event.set()
        self._is_connected = False
        self.join()

    def get_motion_state(self):
        me = self.motion_event
        state = np.array(me.translation + me.rotation, 
            dtype=self.dtype) / self.max_value
        is_dead = (-self.deadzone < state) & (state < self.deadzone)
        state[is_dead] = 0
        return state
    
    def get_motion_state_transformed(self):
        """
        Return in right-handed coordinate
        z
        *------>y right
        |   _
        |  (O) space mouse
        v
        x
        back

        """
        state = self.get_motion_state()
        tf_state = np.zeros_like(state)
        tf_state[:3] = self.tx_zup_spnav @ state[:3]
        tf_state[3:] = self.tx_zup_spnav @ state[3:]
        return tf_state

    def is_button_pressed(self, button_id):
        return self.button_state[button_id]

    def run(self):
        spnav_open()
        self._is_connected = True
        try:
            while not self.stop_event.is_set():
                event = spnav_poll_event()
                if isinstance(event, SpnavMotionEvent):
                    self.motion_event = event
                elif isinstance(event, SpnavButtonEvent):
                    self.button_state[event.bnum] = event.press
                else:
                    time.sleep(1/200)
        finally:
            self._is_connected = False
            spnav_close()

    # delta action
    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(
                "SpaceMouseTeleop is not connected. You need to run `connect()` before `get_action()`."
            )

        # self._drain_pressed_keys()
        sm_state = self.get_motion_state_transformed()

        dpos = sm_state[:3] * self.max_pos_speed / self.frequency
            
        # Currently No rotation operation 
        # drot_xyz = sm_state[3:] * (max_rot_speed / frequency)
        
        # if not self.is_button_pressed(0):
        #     # translation mode
        #     drot_xyz[:] = 0
        # else:
        #     dpos[:] = 0
        # if not self.is_button_pressed(1):
        
        # X-Y 2D translation mode, no gripper control. Modify the code if you need more DOF control
        dpos[2] = 0    

        gripper_action = 1.0

        # output is delta change of the robot pose
        action_dict = {
            "pose.dx": dpos[0],
            "pose.dy": dpos[1],
            "pose.dz": dpos[2],
        }

        if self.config.use_gripper:
            action_dict.update({"gripper.pos": gripper_action})

        return action_dict

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError