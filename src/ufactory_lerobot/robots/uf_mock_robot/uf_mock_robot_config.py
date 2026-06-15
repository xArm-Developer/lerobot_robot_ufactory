from dataclasses import dataclass, field
from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots import RobotConfig

@RobotConfig.register_subclass("uf::mock_robot")
@dataclass
class UFMockRobotConfig(RobotConfig):
    # cameras
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "fisheye": OpenCVCameraConfig(
                index_or_path=6,
                width=640,
                height=480,
                fps=30,
                fourcc="MJPG"
            )
        }
    )

    robot_dof: int | None = None  # Set it correctly if controlling in joint space!
    control_space: str = "joint"
    gripper_type: int = 1           # 1: xArm Gripper, 10: Pika Gripper
    observe_joint_vel: bool = False # only effective in joint control mode
    teleop:  None = None # from lerobot.teleoperators import Teleoperator
    state_offset_action: int = 3    # the number of previous teleop actions to be included in the observation

    def __post_init__(self):
        super().__post_init__()
        self.id = 'uf_mock_robot' if self.id is None else self.id
