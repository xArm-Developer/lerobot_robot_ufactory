from lerobot.teleoperators import Teleoperator
from lerobot_robot_ufactory.context import register_teleop, unregister_teleop


class UFBaseTeleop(Teleoperator):
    config_class = None
    name = "Base Teleop For UFACTORY"

    def __init__(self, config):
        super().__init__(config)
        self.config = config

    def connect(self, calibrate: bool = False) -> None:
        register_teleop(self)

    def disconnect(self):
        unregister_teleop(self)

    def set_teleop_enabled(self, enabled: bool, obs=None):
        """
        启用/停用遥操作
        当enabled为True且obs不为None时, 顺便设置机械臂初始位置映射
        """
        pass
