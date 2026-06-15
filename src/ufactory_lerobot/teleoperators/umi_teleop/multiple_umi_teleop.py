#!/usr/bin/env python

from typing import Any
from .multiple_umi_teleop_config import MultipleUmiTeleopConfig
from ..base_teleop import UFBaseTeleop
from .umi_teleop import UmiTeleop


class MultipleUmiTeleop(UFBaseTeleop):
    
    config_class = MultipleUmiTeleopConfig
    name = "Multiple UMI Teleop For xArm"

    def __init__(self, config: MultipleUmiTeleopConfig):        
        super().__init__(config)
        self.config = config
        self.teleops = {}
        for key, teleop_config in self.config.teleops.items():
            self.teleops[key] = UmiTeleop(teleop_config, prefix=key)

    def action_features(self) -> dict:
        action_features = {}
        for teleop in self.teleops.values():
            action_features.update(teleop.action_features)
        return action_features

    @property
    def feedback_features(self) -> dict:
        feedback_features = {}
        for teleop in self.teleops.values():
            feedback_features.update(teleop.feedback_features)
        return feedback_features

    @property
    def is_connected(self) -> bool:
        return all(teleop.is_connected for teleop in self.teleops.values())

    @property
    def is_calibrated(self) -> bool:
        return all(teleop.is_calibrated for teleop in self.teleops.values())

    def connect(self, calibrate: bool = True) -> None:
        for teleop in self.teleops.values():
            teleop.connect(calibrate=calibrate)

    def calibrate(self) -> None:
        for teleop in self.teleops.values():
            teleop.calibrate()

    def configure(self) -> None:
        for teleop in self.teleops.values():
            teleop.configure()
    
    def disconnect(self) -> None:
        for teleop in self.teleops.values():
            teleop.disconnect()

    def set_teleop_enabled(self, enabled: bool, obs=None):
        for key, teleop in self.teleops.items():
            if obs is not None:
                teleop_obs = {k: v for k, v in obs.items() if k.startswith(f"{key}.")}
            else:
                teleop_obs = None
            teleop.set_teleop_enabled(enabled, teleop_obs)

    def get_action(self) -> dict[str, Any]:
        actions = {}
        for teleop in self.teleops.values():
            actions.update(teleop.get_action())
        return actions

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        for key, teleop in self.teleops.items():
            feedback_subset = {k: v for k, v in feedback.items() if k.startswith(f"{key}.")}
            teleop.send_feedback(feedback_subset)
