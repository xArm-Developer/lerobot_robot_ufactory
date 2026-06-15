import logging
import importlib
from lerobot.utils.control_utils import is_headless

# recursive call, inspired from gello_software:
def instantiate_from_dict(cfg, ignore_cameras=False):
    """Instantiate objects from configuration."""
    if isinstance(cfg, dict) and "_target_" in cfg:
        module_path, class_name = cfg["_target_"].rsplit(".", 1)
        cls = getattr(importlib.import_module(module_path), class_name)
        kwargs = {k: v for k, v in cfg.items() if k != "_target_"}
        # pp_dict ={k: instantiate_from_dict(v, ignore_cameras) for k, v in kwargs.items()} 
        # print(pp_dict)
        return cls(**{k: {} if ignore_cameras and k == 'cameras' else instantiate_from_dict(v, ignore_cameras) for k, v in kwargs.items()})
    elif isinstance(cfg, dict):
        return {k: {} if ignore_cameras and k == 'cameras' else instantiate_from_dict(v, ignore_cameras) for k, v in cfg.items()}
    elif isinstance(cfg, list):
        return [instantiate_from_dict(v, ignore_cameras) for v in cfg]
    else:
        return cfg

def init_keyboard_listener(events: dict = None, on_press: callable = None, on_release: callable = None):
    """
    Initializes a non-blocking keyboard listener for real-time user interaction.

    This function sets up a listener for specific keys (right arrow, left arrow, escape) to control
    the program flow during execution, such as stopping recording or exiting loops. It gracefully
    handles headless environments where keyboard listening is not possible.

    Returns:
        A tuple containing:
        - The `pynput.keyboard.Listener` instance, or `None` if in a headless environment.
        - A dictionary of event flags (e.g., `exit_early`) that are set by key presses.
    """
    # Allow to exit early while recording an episode or resetting the environment,
    # by tapping the right arrow key '->'. This might require a sudo permission
    # to allow your terminal to monitor keyboard events.
    if events is None:
        events = {}
        events["exit_early"] = False
        events["rerecord_episode"] = False
        events["stop_recording"] = False

    if is_headless():
        logging.warning(
            "Headless environment detected. On-screen cameras display and keyboard inputs will not be available."
        )
        listener = None
        return listener, events

    # Only import pynput if not in a headless environment
    from pynput import keyboard

    if on_press is None:
        def on_press(key):
            try:
                if key == keyboard.Key.right:
                    print("Right arrow key pressed. Exiting loop...")
                    events["exit_early"] = True
                elif key == keyboard.Key.left:
                    print("Left arrow key pressed. Exiting loop and rerecord the last episode...")
                    events["rerecord_episode"] = True
                    events["exit_early"] = True
                elif key == keyboard.Key.esc:
                    print("Escape key pressed. Stopping data recording...")
                    events["stop_recording"] = True
                    events["exit_early"] = True
            except Exception as e:
                print(f"Error handling key press: {e}")

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    return listener, events
