import yaml
import argparse
import logging
import time
from pathlib import Path
from dataclasses import asdict, dataclass
from pprint import pformat
import ufactory_lerobot # patch
from lerobot.scripts.lerobot_record import register_third_party_plugins
from lerobot.processor import (
    make_default_processors,
)
from lerobot.robots import (  # noqa: F401
    RobotConfig,
    make_robot_from_config,
)
from lerobot.teleoperators import (  # noqa: F401
    TeleoperatorConfig,
    make_teleoperator_from_config,
)
from lerobot.utils.control_utils import (
    is_headless,
    init_keyboard_listener
)
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import (
    init_logging,
)
from ufactory_lerobot.utils.utils import instantiate_from_dict

@dataclass
class TeleopConfig:
    robot: RobotConfig
    teleop: TeleoperatorConfig
    fps: int = 30


def teleop_loop(cfg: TeleopConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))

    teleop = make_teleoperator_from_config(cfg.teleop)
    if hasattr(cfg.robot, "teleop"):
        cfg.robot.teleop = teleop
    robot = make_robot_from_config(cfg.robot)

    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    robot.connect()
    teleop.connect()

    events = {"exit": False}
    listener = None

    if not is_headless():
        from pynput import keyboard

        def on_press(key):
            try:
                if key == keyboard.Key.esc:
                    print("Escape key pressed. Stopping ...")
                    events["exit"] = True
            except Exception as e:
                print(f"Error handling key press: {e}")

        listener, events = init_keyboard_listener(events=events, on_press=on_press)

    sleep_time_s = 1 / cfg.fps

    print("\n********** Test Teleop With Robot **********")
    input('Enter to control robot with teleop >>> ')

    print("\n********** Teleop Control Loop Start **********")

    while not events["exit"]:
        start_loop_t = time.perf_counter()

        # Get robot observation
        obs = robot.get_observation()

        act = teleop.get_action()
        act_processed_teleop = teleop_action_processor((act, obs))

        robot_action_to_send = robot_action_processor((act_processed_teleop, obs))
        robot.send_action(robot_action_to_send)

        dt_s = time.perf_counter() - start_loop_t
        precise_sleep(sleep_time_s - dt_s)
    
    print("\n********** Teleop Control Loop Exit **********")
    robot.disconnect()
    teleop.disconnect()
    if not is_headless() and listener is not None:
        listener.stop()

def main():
    parser = argparse.ArgumentParser(description='configuration args')
    parser.add_argument('-c', '--config', type=str, required=True,
                       help='configuration file path, e.g.my_config.yaml')
    parser.add_argument('-f', '--fps', type=int, default=30,
                       help='control loop frequency in Hz (default: 30)')
    args = parser.parse_args()
    try:
        with open(Path(args.config).expanduser(), 'r') as f:
            cfg = yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading config yaml file: {e}")
    else:
        register_third_party_plugins()
        config = instantiate_from_dict(cfg, ignore_cameras=True)

        teleop_cfg = TeleopConfig(robot=config["RobotConfig"], teleop=config["TeleoperatorConfig"], fps=args.fps)
        teleop_loop(teleop_cfg)


if __name__ == "__main__":
    main()
