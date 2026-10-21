"""rclpy / ROS 消息包的 sys.modules 桩（Phase 4 测试用）。

让 pico_elevationmap_ros.node 在无 ROS 环境可 import、可实例化：桩 Node 基类
记录 create_* 调用；消息类为宽松构造的占位类（转换逻辑在 messages.py 按鸭子
类型解包，不依赖真实消息类）。install() 幂等；返回被打桩的模块名列表。
"""

from __future__ import annotations

import sys
from types import ModuleType, SimpleNamespace


class Flex:
    """宽松构造基类：任意关键字属性。"""

    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class _TriggerResponse(Flex):
    def __init__(self, success: bool = False, message: str = "", **kwargs):
        super().__init__(success=success, message=message, **kwargs)


class Trigger(Flex):
    class Request(Flex):
        pass

    class Response(_TriggerResponse):
        pass


class _StubNode:
    """rclpy.node.Node 的最小桩：记录注册调用，可按 topic/name 查找。"""

    def __init__(self, name: str, **kwargs):
        self.node_name = name
        self.subscriptions: list[SimpleNamespace] = []
        self.services: list[SimpleNamespace] = []
        self.timers: list[SimpleNamespace] = []
        self.parameters: list[tuple[str, object]] = []

    def declare_parameter(self, name: str, default=None):
        self.parameters.append((name, default))
        return SimpleNamespace(value=default)

    def create_subscription(self, msg_type, topic, callback, qos):
        sub = SimpleNamespace(msg_type=msg_type, topic=topic, callback=callback, qos=qos)
        self.subscriptions.append(sub)
        return sub

    def create_publisher(self, msg_type, topic, qos):
        pub = SimpleNamespace(msg_type=msg_type, topic=topic, qos=qos, published=[])
        pub.publish = lambda msg: pub.published.append(msg)
        return pub

    def create_service(self, srv_type, name, callback):
        srv = SimpleNamespace(srv_type=srv_type, name=name, callback=callback)
        self.services.append(srv)
        return srv

    def create_timer(self, period, callback):
        timer = SimpleNamespace(period=period, callback=callback)
        self.timers.append(timer)
        return timer

    def get_logger(self):
        return SimpleNamespace(info=lambda *a, **k: None, warn=lambda *a, **k: None)


def install() -> list[str]:
    """把 rclpy / 消息包桩注入 sys.modules（幂等）。返回桩模块名列表。"""
    installed: list[str] = []

    def _mod(name: str, **attrs):
        mod = sys.modules.get(name) or ModuleType(name)
        for k, v in attrs.items():
            setattr(mod, k, v)
        sys.modules[name] = mod
        if name not in installed:
            installed.append(name)

    _mod("rclpy", init=lambda *a, **k: None, spin=lambda *a, **k: None, shutdown=lambda *a, **k: None)
    _mod("rclpy.node", Node=_StubNode)
    sys.modules["rclpy"].node = sys.modules["rclpy.node"]
    for pkg, classes in (
        ("sensor_msgs", {"msg": {"PointCloud2": Flex, "PointField": Flex}}),
        ("geometry_msgs", {"msg": {"PoseWithCovarianceStamped": Flex, "Pose": Flex, "Point": Flex, "Quaternion": Flex}}),
        ("nav_msgs", {"msg": {"Odometry": Flex}}),
        ("std_srvs", {"srv": {"Trigger": Trigger}}),
    ):
        _mod(pkg, __path__=[])
        for sub, subcls in classes.items():
            _mod(f"{pkg}.{sub}", **subcls)
            setattr(sys.modules[pkg], sub, sys.modules[f"{pkg}.{sub}"])
    return installed
