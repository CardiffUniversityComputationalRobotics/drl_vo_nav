"""Minimal rospy-like compatibility helpers backed by rclpy.

This module exists to keep the training environment logic close to the
original implementation while running on ROS 2.
"""

import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

import rclpy
from rclpy.duration import Duration as RclpyDuration
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile


class ROSException(Exception):
    """Compatibility exception for ROS API errors."""


class ServiceException(Exception):
    """Compatibility exception for ROS service call errors."""


WARN = 30
ERROR = 40


@dataclass
class _PublisherWrapper:
    pub: Any
    name: str

    def publish(self, msg: Any) -> None:
        self.pub.publish(msg)

    def get_num_connections(self) -> int:
        return self.pub.get_subscription_count()


class _ServiceProxy:
    def __init__(self, node: Any, name: str, srv_type: Any):
        self._node = node
        self._name = name
        self._srv_type = srv_type
        self._client = node.create_client(srv_type, name)

    def wait_for_service(self, timeout_sec: Optional[float] = None) -> bool:
        if timeout_sec is None:
            while not self._client.wait_for_service(timeout_sec=1.0):
                if not rclpy.ok():
                    return False
            return True
        return self._client.wait_for_service(timeout_sec=timeout_sec)

    def __call__(self, *args: Any) -> Any:
        req = self._srv_type.Request()
        if len(args) == 1 and isinstance(args[0], self._srv_type.Request):
            req = args[0]
        elif args:
            fields = list(req.get_fields_and_field_types().keys())
            for key, value in zip(fields, args):
                setattr(req, key, value)

        if not self.wait_for_service(timeout_sec=5.0):
            raise ServiceException(f'Service {self._name} not available')

        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(_get_node(), future)
        if not future.done() or future.result() is None:
            raise ServiceException(f'Service {self._name} call failed')
        return future.result()


class _Rate:
    def __init__(self, hz: float):
        self._period = 0.0 if hz <= 0.0 else 1.0 / hz

    def sleep(self) -> None:
        if self._period > 0.0:
            time.sleep(self._period)


class Time:
    @staticmethod
    def now() -> Any:
        return _get_node().get_clock().now().to_msg()


class Duration:
    def __init__(self, secs: float = 0.0):
        self._duration = RclpyDuration(seconds=secs)


_node = None
_executor = None
_service_proxies: Dict[str, _ServiceProxy] = {}


def _ensure_rclpy() -> None:
    if not rclpy.ok():
        rclpy.init(args=None)


def _get_node() -> Any:
    global _node
    if _node is None:
        init_node('drl_vo_compat')
    return _node


def init_node(name: str, anonymous: bool = False, log_level: Optional[int] = None) -> Any:
    del log_level
    global _node, _executor
    _ensure_rclpy()

    if _node is not None:
        return _node

    if anonymous:
        name = f'{name}_{int(time.time() * 1000)}'

    _node = rclpy.create_node(name)
    _executor = SingleThreadedExecutor()
    _executor.add_node(_node)
    return _node


def get_param(name: str, default: Any = None) -> Any:
    node = _get_node()
    param_name = name[1:] if name.startswith('~') else name
    if not node.has_parameter(param_name):
        node.declare_parameter(param_name, default)
    return node.get_parameter(param_name).value


def is_shutdown() -> bool:
    return not rclpy.ok()


def signal_shutdown(reason: str = '') -> None:
    if reason:
        loginfo(reason)
    if rclpy.ok():
        rclpy.shutdown()


def spin() -> None:
    rclpy.spin(_get_node())


def get_time() -> float:
    now = _get_node().get_clock().now()
    return now.nanoseconds / 1e9


def sleep(duration: float) -> None:
    time.sleep(duration)


def logdebug(msg: str, *args: Any) -> None:
    _get_node().get_logger().debug(msg % args if args else msg)


def loginfo(msg: str, *args: Any) -> None:
    _get_node().get_logger().info(msg % args if args else msg)


def logwarn(msg: str, *args: Any) -> None:
    _get_node().get_logger().warn(msg % args if args else msg)


def logerr(msg: str, *args: Any) -> None:
    _get_node().get_logger().error(msg % args if args else msg)


def logfatal(msg: str, *args: Any) -> None:
    _get_node().get_logger().fatal(msg % args if args else msg)


def Subscriber(name: str, msg_type: Any, callback: Any, queue_size: int = 10, buff_size: Optional[int] = None) -> Any:
    del buff_size
    return _get_node().create_subscription(msg_type, name, callback, queue_size)


def Publisher(name: str, msg_type: Any, queue_size: int = 10, latch: bool = False) -> _PublisherWrapper:
    if latch:
        qos = QoSProfile(depth=queue_size, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    else:
        qos = QoSProfile(depth=queue_size)
    pub = _get_node().create_publisher(msg_type, name, qos)
    return _PublisherWrapper(pub=pub, name=name)


def ServiceProxy(name: str, srv_type: Any) -> _ServiceProxy:
    proxy = _ServiceProxy(_get_node(), name, srv_type)
    _service_proxies[name] = proxy
    return proxy


def wait_for_service(name: str, timeout: Optional[float] = None) -> None:
    end_time = None if timeout is None else (time.time() + timeout)
    node = _get_node()

    while rclpy.ok():
        names = {srv_name for srv_name, _ in node.get_service_names_and_types()}
        if name in names:
            return
        if end_time is not None and time.time() >= end_time:
            raise ROSException(f'Service {name} unavailable')
        time.sleep(0.1)

    raise ROSException(f'Service {name} unavailable')


def wait_for_message(name: str, msg_type: Any, timeout: Optional[float] = None) -> Any:
    node = _get_node()
    fut = rclpy.task.Future()

    def _cb(msg: Any) -> None:
        if not fut.done():
            fut.set_result(msg)

    sub = node.create_subscription(msg_type, name, _cb, 10)
    try:
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        if not fut.done():
            raise ROSException(f'Topic {name} is not available')
        return fut.result()
    finally:
        node.destroy_subscription(sub)


def Rate(hz: float) -> _Rate:
    return _Rate(hz)
