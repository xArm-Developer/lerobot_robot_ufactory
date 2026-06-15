from dataclasses import dataclass
from lerobot.robots import RobotConfig
from .uf_mock_robot_config import UFMockRobotConfig

@RobotConfig.register_subclass("uf::multiple_mock_robot")
@dataclass
class MultipleUFMockRobotConfig(RobotConfig):
    robots: dict[str, UFMockRobotConfig]

    def __post_init__(self):
        super().__post_init__()
        self.id = 'multiple_uf_mock_robot' if self.id is None else self.id
