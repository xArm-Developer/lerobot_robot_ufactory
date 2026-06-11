from dataclasses import dataclass
from lerobot.robots import RobotConfig
from .uf_mock_robot_config import UFMockRobotConfig

@RobotConfig.register_subclass("uf::multiple_mock_robot")
@dataclass
class MultipleUFMockRobotConfig(RobotConfig):
    robots: dict[str, UFMockRobotConfig]
