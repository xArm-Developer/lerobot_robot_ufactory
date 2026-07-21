import os

# register plugin
import lerobot_robot_ufactory.cameras.umi_camera
import lerobot_robot_ufactory.robots.uf_robot
import lerobot_robot_ufactory.robots.uf_mock_robot
import lerobot_robot_ufactory.teleoperators.uf_mock_teleop
import lerobot_robot_ufactory.teleoperators.gello_teleop
import lerobot_robot_ufactory.teleoperators.pika_teleop
import lerobot_robot_ufactory.teleoperators.space_mouse
import lerobot_robot_ufactory.teleoperators.umi_teleop

def patch_lerobot_modules():
    """
    Patch lerobot modules to use uFactory robot implementations.
    This function replaces the default implementations of certain functions in the lerobot package
    with the uFactory-specific implementations. It is intended to be called at the beginning of a script
    that uses the lerobot package, before any other imports from lerobot are made.
    """
    from lerobot_robot_ufactory.cameras.utils import make_cameras_from_configs as _uf_make_cameras_from_configs
    from lerobot_robot_ufactory.robots.utils import make_robot_from_config as _uf_make_robot_from_config
    from lerobot_robot_ufactory.teleoperators.utils import make_teleoperator_from_config as _uf_make_teleoperator_from_config
    from lerobot_robot_ufactory.configs.parser import wrap as _uf_config_parser_wrap
    import lerobot.cameras as _lerobot_cameras
    import lerobot.robots as _lerobot_robot
    import lerobot.teleoperators as _lerobot_teleoperators
    import lerobot.cameras.utils as _lerobot_cameras_utils
    import lerobot.robots.utils as _lerobot_robot_utils
    import lerobot.teleoperators.utils as _lerobot_teleoperators_utils
    import lerobot.configs.parser as _lerobot_configs_parser
    # patch
    _lerobot_cameras.make_cameras_from_configs = _uf_make_cameras_from_configs
    _lerobot_robot.make_robot_from_config = _uf_make_robot_from_config
    _lerobot_teleoperators.make_teleoperator_from_config = _uf_make_teleoperator_from_config
    _lerobot_cameras_utils.make_cameras_from_configs = _uf_make_cameras_from_configs
    _lerobot_robot_utils.make_robot_from_config = _uf_make_robot_from_config
    _lerobot_teleoperators_utils.make_teleoperator_from_config = _uf_make_teleoperator_from_config
    _lerobot_configs_parser.wrap = _uf_config_parser_wrap

_UF_LEROBOT_PATCH_TYPE = os.environ.get('UF_LEROBOT_PATCH_TYPE', '0')
if _UF_LEROBOT_PATCH_TYPE == '1':
    print("Using UF_LEROBOT_PATCH_TYPE=1: patching lerobot modules for UFACTORY robot support.")
    patch_lerobot_modules()

# # ── 重新装饰上游脚本中已用旧 wrap 装饰过的函数 ──
# # 因为 @parser.wrap() 在模块 import 时就执行了，
# # register_third_party_plugins() 触发本文件时 record() 已经是旧装饰器包好的。
# # 取出原始函数，用 _uf_config_wrap 重新包一次。
# import sys as _sys
# # 执行 python xxx.py 时模块名是 __main__，console_scripts 入口时是完整路径
# _SCRIPTS_TO_REWRAP = (
#     "__main__",
#     "lerobot.scripts.lerobot_record",
#     "lerobot.scripts.lerobot_teleoperate",
#     "lerobot.scripts.lerobot_eval",
# )
# for _mod_name in _SCRIPTS_TO_REWRAP:
#     _mod = _sys.modules.get(_mod_name)
#     if _mod is None:
#         continue
#     _record_fn = getattr(_mod, "record", getattr(_mod, "teleoperate", getattr(_mod, "eval_main", None)))
#     if _record_fn is None:
#         continue
#     _original = getattr(_record_fn, "__wrapped__", None)
#     if _original is not None:
#         if _mod_name == "__main__":
#             _sys.modules[_mod_name] = _uf_config_wrap()(_original)
#         else:
#             setattr(_mod, _record_fn.__name__, _uf_config_wrap()(_original))
#         break  # 找到并处理后就退出，避免重复
