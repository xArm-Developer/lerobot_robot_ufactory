"""上下文变量，用于跨模块共享运行时对象，避免属性注入。"""
import contextvars

_active_teleops: contextvars.ContextVar = contextvars.ContextVar("active_teleops", default={})


def register_teleop(teleop) -> None:
    """注册 teleop, 以 teleop.id 为 key"""
    teleops = _active_teleops.get()
    teleops[teleop.id] = teleop
    _active_teleops.set(teleops)


def unregister_teleop(teleop) -> None:
    """移除 teleop"""
    teleops = _active_teleops.get()
    teleops.pop(teleop.id, None)
    _active_teleops.set(teleops)


def get_active_teleop(teleop_id: str | None = None):
    """获取 active teleop。不传 id 返回第一个；传 id 返回对应 teleop。"""
    teleops = _active_teleops.get()
    if not teleops:
        return None
    if teleop_id is not None:
        return teleops.get(teleop_id)
    return next(iter(teleops.values()))
