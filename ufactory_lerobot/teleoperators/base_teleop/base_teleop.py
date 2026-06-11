from lerobot.teleoperators import Teleoperator


class UFBaseTeleop(Teleoperator):
    config_class = None
    name = "Base Teleop For UFACTORY"

    def __init__(self, config):
        super().__init__(config)
        self.config = config

    def set_ctrl_status(self, status):
        pass
