from dataclasses import dataclass, field
from typing import Tuple
from lerobot.cameras import CameraConfig
from lerobot.cameras.realsense import RealSenseCameraConfig
from lerobot.robots import RobotConfig

@RobotConfig.register_subclass("uf::robot")
@dataclass
class UFRobotConfig(RobotConfig):
    # cameras
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "overhead": RealSenseCameraConfig(
                serial_number_or_name="Intel RealSense D435I",
                fps=30,
                width=640, # 1280
                height=480, # 720
                # rotation=90,
            ),
            "tool": RealSenseCameraConfig(
                serial_number_or_name="Intel RealSense D435",
                fps=30,
                width=640, # 1280
                height=480, # 720
            ),
        }
    )

    robot_ip: str = "192.168.1.127"
    robot_dof: int | None = None  # Set it correctly if controlling in joint space!
    control_space: str = "joint"
    gripper_type: int = 1       # 1: xArm Gripper, 2: xArm Gripper G2, 10: Pika Gripper, 11: Robotiq 2F-85
    gripper_port: str = None    # only used by pika gripper (gripper_type=10)
    gripper_speed: int = -1     # auto
    gripper_force: int = -1     # auto
    observe_joint_vel: bool = False # only effective in joint control mode
    start_joints: Tuple[float, ...] = (0, 0, 0, 90, 0, 90, 0) # °
    start_tcp_pose: Tuple[float, ...] = None # [x, y, z, roll(°), pitch(°), yaw(°)]
    max_joint_velocity: int = 90   # °/s, only effective in joint control mode
    max_linear_velocity: int = 200 # mm/s, only effective in cartesian control mode
    no_action: bool = False # only for debug

    def __post_init__(self):
        super().__post_init__()
        self.id = 'uf_robot' if self.id is None else self.id
