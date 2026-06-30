#!/usr/bin/env python

from lerobot.processor.core import RobotAction, RobotObservation
from lerobot.processor.core import RobotObservation
from lerobot.robots import Robot
from .multiple_uf_mock_robot_config import MultipleUFMockRobotConfig
from .uf_mock_robot import UFMockRobot


class MultipleUFMockRobot(Robot):

    config_class = MultipleUFMockRobotConfig
    name = "UFACTORY Multiple Mock Robot"

    def __init__(self, config: MultipleUFMockRobotConfig):
        super().__init__(config)
        self.config = config
        self.robots: dict[str, UFMockRobot] = {}
        for key, robot_config in self.config.robots.items():
            self.robots[key] = UFMockRobot(robot_config, prefix=key)

        self.cameras = {}
        for robot in self.robots.values():
            self.cameras.update(robot.cameras)

    @property
    def observation_features(self) -> dict:
        observation_features = {}
        for robot in self.robots.values():
            observation_features.update(robot.observation_features)
        return observation_features

    @property
    def action_features(self) -> dict:
        action_features = {}
        for robot in self.robots.values():
            action_features.update(robot.action_features)
        return action_features

    @property
    def is_connected(self) -> bool:
        return all(robot.is_connected for robot in self.robots.values())

    @property
    def is_calibrated(self) -> bool:
        return all(robot.is_calibrated for robot in self.robots.values())

    def connect(self, calibrate: bool = True) -> None:
        for robot in self.robots.values():
            robot.connect(calibrate=calibrate)

    def calibrate(self) -> None:
        for robot in self.robots.values():
            robot.calibrate()

    def configure(self) -> None:
        for robot in self.robots.values():
            robot.configure()

    def disconnect(self) -> None:
        for robot in self.robots.values():
            robot.disconnect()

    def get_observation(self) -> RobotObservation:
        observations = [robot.get_observation() for robot in self.robots.values()]
        combined_observation = RobotObservation()
        for obs in observations:
            combined_observation.update(obs)
        return combined_observation

    def send_action(self, action: RobotAction) -> RobotAction:
        for key, robot in self.robots.items():
            action_subset = {k: v for k, v in action.items() if k.startswith(f"{key}.")}
            robot.send_action(action_subset)
        return action
