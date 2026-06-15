import yaml
import time
import argparse
from pathlib import Path
import ufactory_lerobot # patch
from lerobot.scripts.lerobot_record import *
from ufactory_lerobot.teleoperators.uf_mock_teleop import UFMockTeleop
from ufactory_lerobot.teleoperators.base_teleop import UFBaseTeleop
from ufactory_lerobot.utils.utils import instantiate_from_dict, init_keyboard_listener
    

@safe_stop_image_writer
def record_loop(
    robot: Robot,
    events: dict,
    fps: int,
    teleop_action_processor: RobotProcessorPipeline[
        tuple[RobotAction, RobotObservation], RobotAction
    ],  # runs after teleop
    robot_action_processor: RobotProcessorPipeline[
        tuple[RobotAction, RobotObservation], RobotAction
    ],  # runs before robot
    robot_observation_processor: RobotProcessorPipeline[
        RobotObservation, RobotObservation
    ],  # runs after robot
    dataset: LeRobotDataset | None = None,
    teleop: Teleoperator | list[Teleoperator] | None = None,
    policy: PreTrainedPolicy | None = None,
    preprocessor: PolicyProcessorPipeline[dict[str, Any], dict[str, Any]] | None = None,
    postprocessor: PolicyProcessorPipeline[PolicyAction, PolicyAction] | None = None,
    control_time_s: int | None = None,
    single_task: str | None = None,
    display_data: bool = False,
    display_compressed_images: bool = False,
    frame_callback: callable = None,
):
    if dataset is not None and dataset.fps != fps:
        raise ValueError(f"The dataset fps should be equal to requested fps ({dataset.fps} != {fps}).")

    teleop_arm = teleop_keyboard = None
    if isinstance(teleop, list):
        teleop_keyboard = next((t for t in teleop if isinstance(t, KeyboardTeleop)), None)
        teleop_arm = next(
            (
                t
                for t in teleop
                if isinstance(
                    t,
                    (
                        so_leader.SO100Leader
                        | so_leader.SO101Leader
                        | koch_leader.KochLeader
                        | omx_leader.OmxLeader
                    ),
                )
            ),
            None,
        )

        if not (teleop_arm and teleop_keyboard and len(teleop) == 2 and robot.name == "lekiwi_client"):
            raise ValueError(
                "For multi-teleop, the list must contain exactly one KeyboardTeleop and one arm teleoperator. Currently only supported for LeKiwi robot."
            )

    # Reset policy and processor if they are provided
    if policy is not None and preprocessor is not None and postprocessor is not None:
        policy.reset()
        preprocessor.reset()
        postprocessor.reset()

    last_robot_cmd = robot.get_observation()
    # only positional cmd for now: Remove velo from observation for cmd if needed!
    last_robot_cmd = { k: v for k,v in last_robot_cmd.items() if not "vel" in k }

    timestamp = 0
    start_episode_t = time.perf_counter()
    while timestamp < control_time_s:
        start_loop_t = time.perf_counter()

        if events["exit_early"]:
            events["exit_early"] = False
            break

        # Get robot observation
        obs = robot.get_observation()

        # Applies a pipeline to the raw robot observation, default is IdentityProcessor
        obs_processed = robot_observation_processor(obs)

        if policy is not None or dataset is not None:
            observation_frame = build_dataset_frame(dataset.features, obs_processed, prefix=OBS_STR)

        # Get action from either policy or teleop
        if policy is not None and preprocessor is not None and postprocessor is not None:
            action_values = predict_action(
                observation=observation_frame,
                policy=policy,
                device=get_safe_torch_device(policy.config.device),
                preprocessor=preprocessor,
                postprocessor=postprocessor,
                use_amp=policy.config.use_amp,
                task=single_task,
                robot_type=robot.robot_type,
            )

            act_processed_policy: RobotAction = make_robot_action(action_values, dataset.features)

        elif policy is None and isinstance(teleop, Teleoperator):
            act = teleop.get_action()

            # (space mouse) from delta Cartesian cmd to absolute command
            if "pose.dx" in act:
                last_robot_cmd.update({"pose.x": last_robot_cmd["pose.x"] + act["pose.dx"], "pose.y": last_robot_cmd["pose.y"] + act["pose.dy"], "pose.z": last_robot_cmd["pose.z"] + act["pose.dz"]})
                act = last_robot_cmd.copy() # watch out this is shallow copy, not for nested dict

            # Applies a pipeline to the raw teleop action, default is IdentityProcessor
            act_processed_teleop = teleop_action_processor((act, obs))

        elif policy is None and isinstance(teleop, list):
            arm_action = teleop_arm.get_action()
            arm_action = {f"arm_{k}": v for k, v in arm_action.items()}
            keyboard_action = teleop_keyboard.get_action()
            base_action = robot._from_keyboard_to_base_action(keyboard_action)
            act = {**arm_action, **base_action} if len(base_action) > 0 else arm_action
            act_processed_teleop = teleop_action_processor((act, obs))
        else:
            logging.info(
                "No policy or teleoperator provided, skipping action generation."
                "This is likely to happen when resetting the environment without a teleop device."
                "The robot won't be at its rest position at the start of the next episode."
            )
            continue

        # Applies a pipeline to the action, default is IdentityProcessor
        if policy is not None and act_processed_policy is not None:
            action_values = act_processed_policy
            robot_action_to_send = robot_action_processor((act_processed_policy, obs))
        else:
            action_values = act_processed_teleop
            robot_action_to_send = robot_action_processor((act_processed_teleop, obs))

        # Send action to robot
        # Action can eventually be clipped using `max_relative_target`,
        # so action actually sent is saved in the dataset. action = postprocessor.process(action)
        # TODO(steven, pepijn, adil): we should use a pipeline step to clip the action, so the sent action is the action that we input to the robot.
        _sent_action = robot.send_action(robot_action_to_send)

        # Write to dataset
        if dataset is not None:
            action_frame = build_dataset_frame(dataset.features, action_values, prefix=ACTION)
            frame = {**observation_frame, **action_frame, "task": single_task}
            if frame_callback is not None:
                frame = frame_callback(frame)
            dataset.add_frame(frame)

        if display_data:
            log_rerun_data(
                observation=obs_processed, action=action_values, compress_images=display_compressed_images
            )

        dt_s = time.perf_counter() - start_loop_t
        precise_sleep(max(1 / fps - dt_s, 0.0))

        timestamp = time.perf_counter() - start_episode_t


@parser.wrap()
def record(cfg: RecordConfig) -> LeRobotDataset:
    init_logging()
    logging.info(pformat(asdict(cfg)))
    if cfg.display_data:
        init_rerun(session_name="recording")

    teleop = make_teleoperator_from_config(cfg.teleop) if cfg.teleop is not None else None
    if hasattr(cfg.robot, "teleop"):
        cfg.robot.teleop = teleop
    robot = make_robot_from_config(cfg.robot)

    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    dataset_features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=teleop_action_processor,
            initial_features=create_initial_features(
                action=robot.action_features
            ),  # TODO(steven, pepijn): in future this should be come from teleop or policy
            use_videos=cfg.dataset.video,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=robot_observation_processor,
            initial_features=create_initial_features(observation=robot.observation_features),
            use_videos=cfg.dataset.video,
        ),
    )

    if cfg.resume:
        dataset = LeRobotDataset(
            cfg.dataset.repo_id,
            root=cfg.dataset.root,
            batch_encoding_size=cfg.dataset.video_encoding_batch_size,
        )

        if hasattr(robot, "cameras") and len(robot.cameras) > 0:
            dataset.start_image_writer(
                num_processes=cfg.dataset.num_image_writer_processes,
                num_threads=cfg.dataset.num_image_writer_threads_per_camera * len(robot.cameras),
            )
        sanity_check_dataset_robot_compatibility(dataset, robot, cfg.dataset.fps, dataset_features)
    else:
        # Create empty dataset or load existing saved episodes
        sanity_check_dataset_name(cfg.dataset.repo_id, cfg.policy)
        dataset = LeRobotDataset.create(
            cfg.dataset.repo_id,
            cfg.dataset.fps,
            root=cfg.dataset.root,
            robot_type=robot.name,
            features=dataset_features,
            use_videos=cfg.dataset.video,
            image_writer_processes=cfg.dataset.num_image_writer_processes,
            image_writer_threads=cfg.dataset.num_image_writer_threads_per_camera * len(robot.cameras),
            batch_encoding_size=cfg.dataset.video_encoding_batch_size,
        )

    # Load pretrained policy
    policy = None if cfg.policy is None else make_policy(cfg.policy, ds_meta=dataset.meta)
    preprocessor = None
    postprocessor = None
    if cfg.policy is not None:
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=cfg.policy,
            pretrained_path=cfg.policy.pretrained_path,
            dataset_stats=rename_stats(dataset.meta.stats, cfg.dataset.rename_map),
            preprocessor_overrides={
                "device_processor": {"device": cfg.policy.device},
                "rename_observations_processor": {"rename_map": cfg.dataset.rename_map},
            },
        )

    robot.connect()
    if teleop is not None:
        teleop.connect()

    is_evt = not is_headless()
    is_uf_teleop = isinstance(teleop, UFBaseTeleop)
    is_recorded = False
    key_dict = {}
    events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}

    if is_evt:
        from pynput import keyboard

        key_dict = {
            keyboard.Key.space: 0,  # start
            keyboard.Key.enter: 0,  # help
        }

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
            if key in key_dict:
                key_dict[key] = True

        def on_release(key):
            try:
                if key == keyboard.Key.enter:
                    if not is_recorded:
                        print('[HELP] <ESC>: EXIT, <SPACE>: START, <LEFT ARROW>: RESET, <RIGH ARROW>: SAVE')
                    else:
                        print('[HELP] <ESC>: EXIT, <LEFT ARROW>: RESET, <RIGH ARROW>: SAVE')
                    # is_recorded = True
            except Exception as e:
                print(f"Error handling key release: {e}")
            if key in key_dict:
                key_dict[key] = False

        listener, events = init_keyboard_listener(events=events, on_press=on_press, on_release=on_release)
        print("\n********** Episode Record Loop Start **********")
        print('[HELP] <ESC>: EXIT, <SPACE>: START, <LEFT ARROW>: RESET, <RIGH ARROW>: SAVE')
    else:
        input('[HELP] Enter to to start record >>> ')
        if is_uf_teleop:
            teleop.set_teleop_enabled(True)
        is_recorded = True
        print("\n********** Episode Record Loop Start **********")

    frame_callback = None

    with VideoEncodingManager(dataset):
        recorded_episodes = 0
        while recorded_episodes < cfg.dataset.num_episodes and not events["stop_recording"]:
            time.sleep(0.01)
            if is_evt:
                if not is_recorded and key_dict[keyboard.Key.space]:
                    is_recorded = True

            if teleop is not None and isinstance(teleop, UFMockTeleop):
                if events["stop_recording"]:
                    continue
                teleop.configure(events=events)
                if events["rerecord_episode"]:
                    events["rerecord_episode"] = False
                    events["exit_early"] = False
                    input('\nPress Enter to regenerate random target location >>>>> ')
                    continue
                if events["stop_recording"]:
                    continue
                is_recorded = True

            if is_recorded:
                events["rerecord_episode"] = False
                events["exit_early"] = False
                if is_uf_teleop:
                    robot.configure()
                    obs = robot.get_observation()
                    teleop.set_teleop_enabled(True, obs)
                log_say(f"Recording episode {dataset.num_episodes}", cfg.play_sounds)
                record_loop(
                    robot=robot,
                    events=events,
                    fps=cfg.dataset.fps,
                    teleop_action_processor=teleop_action_processor,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                    teleop=teleop,
                    policy=policy,
                    preprocessor=preprocessor,
                    postprocessor=postprocessor,
                    dataset=dataset,
                    control_time_s=cfg.dataset.episode_time_s,
                    single_task=cfg.dataset.single_task,
                    display_data=cfg.display_data,
                    frame_callback=frame_callback,
                )
            else:
                continue
            if events['stop_recording']:
                break
            if events["rerecord_episode"]:
                log_say("Re-record episode", cfg.play_sounds)
                events["rerecord_episode"] = False
                events["exit_early"] = False
                if is_uf_teleop:
                    teleop.set_teleop_enabled(False)
                if dataset.episode_buffer:
                    dataset.clear_episode_buffer()
                is_recorded = False
                if is_evt:
                    print('[HELP] <ESC>: EXIT, <SPACE>: START, <LEFT ARROW>: RESET, <RIGH ARROW>: SAVE')
                else:
                    input('\nPress Enter to rerecord this episode >>>>> ')
                    is_recorded = True
                continue

            if is_recorded and not events['stop_recording']:
                log_say(f"Save episode {dataset.num_episodes}", cfg.play_sounds)
                if is_uf_teleop:
                    teleop.set_teleop_enabled(False)
                dataset.save_episode()
                recorded_episodes += 1
                is_recorded = False
                if is_evt:
                    print('[HELP] <ESC>: EXIT, <SPACE>: START, <LEFT ARROW>: RESET, <RIGH ARROW>: SAVE')
                else:
                    input('Press Enter to record at the next episode >>>>> ')
                    is_recorded = True

    print("\n********** Episode Record Loop Exit **********")

    robot.disconnect()
    if teleop is not None:
        teleop.disconnect()

    if is_evt and listener is not None:
        listener.stop()

    if cfg.dataset.push_to_hub:
        dataset.push_to_hub(tags=cfg.dataset.tags, private=cfg.dataset.private)

    log_say("Exiting", cfg.play_sounds)
    return dataset


def main():
    parser = argparse.ArgumentParser(description='configuration args')
    parser.add_argument('-c', '--config', type=str, required=True, 
                       help='configuration file path, e.g.my_config.yaml')
    parser.add_argument('-r', '--resume',
                       action='store_true', # specify --resume if resume needs to be True
                       default=False,
                       help='Whether contitue recording on existing dataset (default: False)')
    args = parser.parse_args()
    try:
        with open(Path(args.config).expanduser(), 'r') as f:
            cfg = yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading config yaml file: {e}")
    else:
        register_third_party_plugins()
        config = instantiate_from_dict(cfg)

        record_cfg = RecordConfig(resume=args.resume, play_sounds=False, robot=config["RobotConfig"], dataset=config["DatasetRecordConfig"], teleop=config["TeleoperatorConfig"])
        record(record_cfg)


if __name__ == "__main__":
    main()
