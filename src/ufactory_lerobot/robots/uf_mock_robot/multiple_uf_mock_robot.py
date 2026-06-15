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
        self.keys = []
        self.robots = []
        for key, robot_config in self.config.robots.items():
            self.keys.append(key)
            self.robots.append(UFMockRobot(robot_config, prefix=key))

    @property
    def observation_features(self) -> dict:
        observation_features = {}
        for robot in self.robots:
            observation_features.update(robot.observation_features)
        return observation_features

    @property
    def action_features(self) -> dict:
        action_features = {}
        for robot in self.robots:
            action_features.update(robot.action_features)
        return action_features

    @property
    def is_connected(self) -> bool:
        return all(robot.is_connected for robot in self.robots)

    @property
    def is_calibrated(self) -> bool:
        return all(robot.is_calibrated for robot in self.robots)

    def connect(self, calibrate: bool = True) -> None:
        for robot in self.robots:
            robot.connect(calibrate=calibrate)

    def calibrate(self) -> None:
        for robot in self.robots:
            robot.calibrate()

    def configure(self) -> None:
        for robot in self.robots:
            robot.configure()

    def disconnect(self) -> None:
        for robot in self.robots:
            robot.disconnect()

    def get_observation(self) -> RobotObservation:
        observations = [robot.get_observation() for robot in self.robots]
        combined_observation = RobotObservation()
        for obs in observations:
            combined_observation.update(obs)
        return combined_observation

    def send_action(self, action: RobotAction) -> RobotAction:
        for i in range(len(self.keys)):
            key = self.keys[i]
            action_subset = {k: v for k, v in action.items() if k.startswith(f"{key}.")}
            self.robots[i].send_action(action_subset)
        return action
