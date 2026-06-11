import importlib

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
