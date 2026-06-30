from dataclasses import dataclass, field
from lerobot.robots import RobotConfig
from .uf_robot_config import UFRobotConfig

@RobotConfig.register_subclass("uf::multiple_robot")
@dataclass
class MultipleUFRobotConfig(RobotConfig):
    robots: dict[str, RobotConfig] = field(
        default_factory=lambda: {}
    )
    async_connect: bool = True
    async_configure: bool = True
    async_action: bool = False

    def __post_init__(self):
        super().__post_init__()
        self.id = 'multiple_uf_robot' if self.id is None else self.id
