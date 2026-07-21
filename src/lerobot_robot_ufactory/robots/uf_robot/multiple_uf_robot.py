#!/usr/bin/env python

import queue
import threading
from typing import Any
from lerobot.robots import Robot
from .multiple_uf_robot_config import MultipleUFRobotConfig
from .uf_robot import UFRobot


class MultipleUFRobot(Robot):

    config_class = MultipleUFRobotConfig
    name = "UFACTORY Multiple Robot"

    def __init__(self, config: MultipleUFRobotConfig):
        super().__init__(config)
        self.config = config
        self._is_async_connect = config.async_connect
        self._is_async_configure = config.async_configure
        self._is_async_action = config.async_action
        self.robots = {}
        self.action_queues = {}
        self.action_threads = {}
        for key, robot_config in self.config.robots.items():
            robot_config.cameras_args = self.config.cameras_args
            robot = UFRobot(robot_config, prefix=key)
            self.robots[key] = robot
            if self._is_async_action:
                action_queue = queue.Queue()
                self.action_queues[key] = action_queue
                action_thread = threading.Thread(target=self.run_action_loop, args=(action_queue, robot), daemon=True)
                self.action_threads[key] = action_thread
                # action_thread.start()
        self.cameras = {}
        for robot in self.robots.values():
            self.cameras.update(robot.cameras)

    def run_action_loop(self, action_queue: queue.Queue, robot: Robot):
        while robot.is_connected:
            try:
                action = action_queue.get(timeout=0.5)
                if action is None:
                    break
                robot.send_action(action)
            except queue.Empty:
                continue

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
        if self._is_async_connect:
            threads = []
            for robot in self.robots.values():
                thread = threading.Thread(target=robot.connect, kwargs={"calibrate": calibrate}, daemon=True)
                threads.append(thread)
                thread.start()
            for thread in threads:
                thread.join()
        else:
            for robot in self.robots.values():
                robot.connect(calibrate=calibrate)
        if self._is_async_action:
            for thread in self.action_threads.values():
                thread.start()

    def calibrate(self) -> None:
        for robot in self.robots.values():
            robot.calibrate()

    def configure(self) -> None:
        if self._is_async_configure:
            threads = []
            for robot in self.robots.values():
                thread = threading.Thread(target=robot.configure, daemon=True)
                threads.append(thread)
                thread.start()
            for thread in threads:
                thread.join()
        else:
            for robot in self.robots.values():
                robot.configure()

    def disconnect(self) -> None:
        for robot in self.robots.values():
            robot.disconnect()

    def get_observation(self) -> dict[str, Any]:    
        observations = {}
        for robot in self.robots.values():
            observations.update(robot.get_observation())
        return observations

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if self._is_async_action:
            for key, robot in self.robots.items():
                action_subset = {k: v for k, v in action.items() if k.startswith(f"{key}.")}
                self.action_queues[key].put(action_subset)
        else:
            for key, robot in self.robots.items():
                action_subset = {k: v for k, v in action.items() if k.startswith(f"{key}.")}
                robot.send_action(action_subset)
        return action
