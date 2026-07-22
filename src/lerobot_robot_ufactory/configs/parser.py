import sys
import tempfile
import yaml
import inspect
from dataclasses import fields, is_dataclass
from types import UnionType
from typing import Union, get_args, get_origin
from lerobot.configs.parser import *


def _is_choice_registry(config_type):
    return (
        isinstance(config_type, type)
        and hasattr(config_type, "get_known_choices")
        and hasattr(config_type, "get_choice_class")
    )


def _unwrap_optional(config_type):
    origin = get_origin(config_type)
    if origin in (Union, UnionType):
        non_none_args = [arg for arg in get_args(config_type) if arg is not type(None)]
        if len(non_none_args) == 1:
            return non_none_args[0]
    return config_type


def _pop_cli_arg(arg_name, args):
    value = None
    filtered_args = []
    prefix = f"--{arg_name}="
    index = 0
    while index < len(args):
        arg = args[index]
        if arg.startswith(prefix):
            value = arg[len(prefix):]
        elif arg == f"--{arg_name}":
            if index + 1 >= len(args):
                raise ValueError(f"Missing value for --{arg_name}")
            value = args[index + 1]
            index += 1
        else:
            filtered_args.append(arg)
        index += 1
    return value, filtered_args

def _filter_unknown_config_fields(data, config_type, resolve_choice=True):
    config_type = _unwrap_optional(config_type)
    origin = get_origin(config_type)

    if origin is list:
        args = get_args(config_type)
        if isinstance(data, list) and args:
            return [_filter_unknown_config_fields(item, args[0]) for item in data]
        return data

    if origin is dict:
        args = get_args(config_type)
        if isinstance(data, dict) and len(args) == 2:
            return {key: _filter_unknown_config_fields(value, args[1]) for key, value in data.items()}
        return data

    if isinstance(data, dict) and _is_choice_registry(config_type) and resolve_choice:
        choice_key = getattr(draccus, "CHOICE_TYPE_KEY", "type")
        choice_name = data.get(choice_key) or data.get("type")
        if choice_name in config_type.get_known_choices():
            choice_type = config_type.get_choice_class(choice_name)
            filtered = _filter_unknown_config_fields(data, choice_type, resolve_choice=False)
            if choice_key in data:
                filtered = {choice_key: data[choice_key], **filtered}
            elif "type" in data:
                filtered = {"type": data["type"], **filtered}
            return filtered
        return data

    if not isinstance(data, dict) or not is_dataclass(config_type):
        return data

    known_fields = {field.name: field for field in fields(config_type)}
    return {
        key: _filter_unknown_config_fields(value, known_fields[key].type)
        for key, value in data.items()
        if key in known_fields
    }

def _filtered_config_path(config_type, config_path):
    if config_path is None:
        return None, None

    config_path = Path(config_path)
    with config_path.open("r", encoding="utf-8") as f:
        raw_config = yaml.safe_load(f) or {}

    filtered_config = _filter_unknown_config_fields(raw_config, config_type)
    if filtered_config == raw_config:
        return config_path, None

    suffix = config_path.suffix if config_path.suffix else ".yaml"
    with tempfile.NamedTemporaryFile("w", suffix=suffix, encoding="utf-8", delete=False) as f:
        yaml.safe_dump(filtered_config, f, sort_keys=False, allow_unicode=True)
        return Path(f.name), Path(f.name)


def wrap(config_path: Path | None = None) -> Callable[[F], F]:
    """
    HACK: Similar to draccus.wrap but does three additional things:
        - Will remove '.path' arguments from CLI in order to process them later on.
        - If a 'config_path' is passed and the main config class has a 'from_pretrained' method, will
          initialize it from there to allow to fetch configs from the hub directly
        - Will load plugins specified in the CLI arguments. These plugins will typically register
            their own subclasses of config classes, so that draccus can find the right class to instantiate
            from the CLI '.type' arguments
    """
    def wrapper_outer(fn: F) -> F:
        @wraps(fn)
        def wrapper_inner(*args: Any, **kwargs: Any) -> Any:
            argspec = inspect.getfullargspec(fn)
            argtype = argspec.annotations[argspec.args[0]]
            if len(args) > 0 and type(args[0]) is argtype:
                cfg = args[0]
                args = args[1:]
            else:
                cli_args = sys.argv[1:]
                plugin_args = parse_plugin_args(PLUGIN_DISCOVERY_SUFFIX, cli_args)
                for plugin_cli_arg, plugin_path in plugin_args.items():
                    try:
                        load_plugin(plugin_path)
                    except PluginLoadError as e:
                        # add the relevant CLI arg to the error message
                        raise PluginLoadError(f"{e}\nFailed plugin CLI Arg: {plugin_cli_arg}") from e
                    cli_args = filter_arg(plugin_cli_arg, cli_args)
                config_path_cli, cli_args = _pop_cli_arg("config_path", cli_args)
                if has_method(argtype, "__get_path_fields__"):
                    path_fields = argtype.__get_path_fields__()
                    cli_args = filter_path_args(path_fields, cli_args)
                if has_method(argtype, "from_pretrained") and config_path_cli:
                    cfg = argtype.from_pretrained(config_path_cli, cli_args=cli_args)
                else:
                    config_path_for_parse = Path(config_path_cli) if config_path_cli else config_path
                    filtered_path, temp_path = _filtered_config_path(argtype, config_path_for_parse)
                    try:
                        cfg = draccus.parse(config_class=argtype, config_path=filtered_path, args=cli_args)
                    finally:
                        if temp_path is not None:
                            temp_path.unlink(missing_ok=True)
            response = fn(cfg, *args, **kwargs)
            return response

        return cast(F, wrapper_inner)

    return cast(Callable[[F], F], wrapper_outer)