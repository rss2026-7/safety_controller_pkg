"""Mock ROS 2 / rclpy stack so `safety_node.py` can be imported unchanged.

The real safety node imports:
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, Float32
    from ackermann_msgs.msg import AckermannDriveStamped

We install fake modules for all of these into sys.modules BEFORE the safety
node is imported, so the safety node sees our minimal but API-compatible
implementations.

The mock provides:
  * `rclpy.init / spin / shutdown` (spin is a no-op — our sim loop drives time)
  * `rclpy.node.Node` with create_publisher / create_subscription /
    create_timer / get_clock / get_logger / declare_parameter / get_parameter
  * In-process pub/sub: publishing to a topic synchronously invokes every
    subscriber callback
  * A manually-advanced clock so we can control "now" from the sim loop
  * LaserScan, Odometry, Bool, Float32, AckermannDriveStamped dataclass-style stand-ins

This is NOT a full rclpy implementation — it has only what safety_node.py
and a simple driver publisher actually need.
"""

from __future__ import annotations

import sys
import time
import types
from dataclasses import dataclass, field
from typing import Any, Callable


# --------------------------------------------------------------------------
# A tiny in-process "broker" that maps topic name -> list of subscriber
# callbacks. Publishers look up the list and invoke the callbacks directly.
# --------------------------------------------------------------------------
class _Broker:
    def __init__(self):
        self.subscribers: dict[str, list[Callable[[Any], None]]] = {}
        # Also remember last published message per topic — useful for the sim
        # loop to read out what the safety node most recently commanded.
        self.last_msg: dict[str, Any] = {}

    def subscribe(self, topic: str, cb: Callable[[Any], None]) -> None:
        self.subscribers.setdefault(topic, []).append(cb)

    def publish(self, topic: str, msg: Any) -> None:
        self.last_msg[topic] = msg
        for cb in self.subscribers.get(topic, []):
            cb(msg)


BROKER = _Broker()


# --------------------------------------------------------------------------
# Manually-advanced clock. The sim loop calls CLOCK.set(t) every tick; every
# rclpy.Time the safety node pulls will reflect that t.
# --------------------------------------------------------------------------
class _SimClock:
    def __init__(self):
        self._seconds: float = 0.0
        self._start_wall: float = time.monotonic()

    def set(self, seconds: float) -> None:
        self._seconds = seconds

    def advance(self, dt: float) -> None:
        self._seconds += dt

    def seconds(self) -> float:
        return self._seconds

    def nanoseconds(self) -> int:
        return int(self._seconds * 1e9)


CLOCK = _SimClock()


# --------------------------------------------------------------------------
# rclpy.Time / Duration mock — supports subtraction to a .nanoseconds-bearing
# object (that's all safety_node.py actually uses).
# --------------------------------------------------------------------------
class _Duration:
    __slots__ = ("nanoseconds",)

    def __init__(self, nanoseconds: int):
        self.nanoseconds = nanoseconds


class _Time:
    __slots__ = ("nanoseconds",)

    def __init__(self, nanoseconds: int):
        self.nanoseconds = int(nanoseconds)

    def __sub__(self, other: "_Time") -> _Duration:
        return _Duration(self.nanoseconds - other.nanoseconds)

    def to_msg(self):
        # Header stamp — a simple object with sec/nanosec fields
        sec = self.nanoseconds // 1_000_000_000
        nanosec = self.nanoseconds % 1_000_000_000
        ns = types.SimpleNamespace(sec=sec, nanosec=nanosec)
        return ns


class _MockClock:
    """The object returned by node.get_clock(). `.now()` reads CLOCK."""

    def now(self) -> _Time:
        return _Time(CLOCK.nanoseconds())


# --------------------------------------------------------------------------
# Parameter mock — stores parameter values for declare_parameter/get_parameter
# --------------------------------------------------------------------------
class Parameter:
    """Mock rclpy.parameter.Parameter for type hints."""
    pass


class _ParameterValue:
    """Mock parameter value with .value attribute."""
    def __init__(self, value: Any):
        self.value = value


# --------------------------------------------------------------------------
# Logger — prints with node name + severity tag. Colors are kept intact
# since safety_node.py embeds ANSI escapes.
# --------------------------------------------------------------------------
class _Logger:
    def __init__(self, name: str):
        self.name = name

    def _emit(self, level: str, msg: str) -> None:
        print(f"[{level:5s}] [{self.name}] {msg}")

    def info(self, msg: str) -> None:
        self._emit("INFO", msg)

    def warn(self, msg: str) -> None:
        self._emit("WARN", msg)

    def warning(self, msg: str) -> None:
        self._emit("WARN", msg)

    def error(self, msg: str) -> None:
        self._emit("ERROR", msg)

    def debug(self, msg: str) -> None:
        # Suppressed by default — uncomment for verbose debugging
        pass


# --------------------------------------------------------------------------
# Publisher / Subscription / Timer mocks
# --------------------------------------------------------------------------
class _Publisher:
    def __init__(self, topic: str, msg_type: type):
        self.topic = topic
        self.msg_type = msg_type

    def publish(self, msg: Any) -> None:
        BROKER.publish(self.topic, msg)


class _Subscription:
    def __init__(self, topic: str, msg_type: type, cb: Callable[[Any], None]):
        self.topic = topic
        self.msg_type = msg_type
        self.cb = cb
        BROKER.subscribe(topic, cb)


@dataclass
class _Timer:
    period: float
    callback: Callable[[], None]
    next_fire: float = 0.0  # in sim seconds

    def tick(self, now_seconds: float) -> None:
        """Called from the sim main loop. Fires the callback as many times
        as needed to catch up to `now_seconds`."""
        if self.next_fire == 0.0:
            self.next_fire = now_seconds + self.period
            return
        while now_seconds >= self.next_fire:
            self.callback()
            self.next_fire += self.period


# Global timer registry, so the sim loop can drive all timers.
TIMERS: list[_Timer] = []


# --------------------------------------------------------------------------
# Node mock — what the safety node actually subclasses.
# --------------------------------------------------------------------------
class Node:
    def __init__(self, node_name: str):
        self._node_name = node_name
        self._logger = _Logger(node_name)
        self._clock = _MockClock()
        self._parameters: dict[str, Any] = {}

    def get_logger(self) -> _Logger:
        return self._logger

    def get_clock(self) -> _MockClock:
        return self._clock

    def create_publisher(self, msg_type: type, topic: str, qos: int) -> _Publisher:
        return _Publisher(topic, msg_type)

    def create_subscription(
        self,
        msg_type: type,
        topic: str,
        cb: Callable[[Any], None],
        qos: int,
    ) -> _Subscription:
        return _Subscription(topic, msg_type, cb)

    def create_timer(self, period: float, cb: Callable[[], None]) -> _Timer:
        t = _Timer(period=period, callback=cb)
        TIMERS.append(t)
        return t

    def declare_parameter(self, name: str, default_value: Any) -> _ParameterValue:
        """Mock declare_parameter - stores the default value."""
        self._parameters[name] = default_value
        return _ParameterValue(default_value)

    def get_parameter(self, name: str) -> _ParameterValue:
        """Mock get_parameter - returns stored value."""
        return _ParameterValue(self._parameters.get(name))


# --------------------------------------------------------------------------
# rclpy module-level functions
# --------------------------------------------------------------------------
_initialized = False


def init(args: list | None = None) -> None:
    global _initialized
    _initialized = True


def shutdown() -> None:
    global _initialized
    _initialized = False


def spin(node: Node) -> None:
    # In the real system, spin is blocking and services all callbacks. In
    # the sim we advance the loop ourselves, so spin is a no-op.
    return


def ok() -> bool:
    return _initialized


# --------------------------------------------------------------------------
# Message-type stand-ins. These just have the fields the safety node reads.
# --------------------------------------------------------------------------
@dataclass
class LaserScan:
    header: Any = None
    angle_min: float = 0.0
    angle_max: float = 0.0
    angle_increment: float = 0.0
    range_min: float = 0.0
    range_max: float = 0.0
    ranges: list = field(default_factory=list)
    intensities: list = field(default_factory=list)


@dataclass
class _Drive:
    speed: float = 0.0
    steering_angle: float = 0.0
    acceleration: float = 0.0
    jerk: float = 0.0
    steering_angle_velocity: float = 0.0


@dataclass
class _Header:
    stamp: Any = None
    frame_id: str = ""


class AckermannDriveStamped:
    """Matches the real ROS message: `.header.stamp` and `.drive.speed` /
    `.drive.steering_angle`."""

    def __init__(self):
        self.header = _Header()
        self.drive = _Drive()


# --------------------------------------------------------------------------
# Additional message types for v2.0 safety controller
# --------------------------------------------------------------------------
@dataclass
class _Twist:
    """geometry_msgs/Twist component for Odometry."""
    linear: Any = None
    angular: Any = None

    def __post_init__(self):
        if self.linear is None:
            self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        if self.angular is None:
            self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


@dataclass
class _TwistWithCovariance:
    """geometry_msgs/TwistWithCovariance for Odometry."""
    twist: _Twist = None
    covariance: list = field(default_factory=lambda: [0.0] * 36)

    def __post_init__(self):
        if self.twist is None:
            self.twist = _Twist()


@dataclass
class _Pose:
    """geometry_msgs/Pose component for Odometry."""
    position: Any = None
    orientation: Any = None

    def __post_init__(self):
        if self.position is None:
            self.position = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        if self.orientation is None:
            self.orientation = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)


@dataclass
class _PoseWithCovariance:
    """geometry_msgs/PoseWithCovariance for Odometry."""
    pose: _Pose = None
    covariance: list = field(default_factory=lambda: [0.0] * 36)

    def __post_init__(self):
        if self.pose is None:
            self.pose = _Pose()


@dataclass
class Odometry:
    """nav_msgs/Odometry — used by safety node to get current velocity."""
    header: _Header = None
    child_frame_id: str = ""
    pose: _PoseWithCovariance = None
    twist: _TwistWithCovariance = None

    def __post_init__(self):
        if self.header is None:
            self.header = _Header()
        if self.pose is None:
            self.pose = _PoseWithCovariance()
        if self.twist is None:
            self.twist = _TwistWithCovariance()


@dataclass
class Bool:
    """std_msgs/Bool — used for external stop requests."""
    data: bool = False


@dataclass
class Float32:
    """std_msgs/Float32 — used for speed limits and status."""
    data: float = 0.0


# --------------------------------------------------------------------------
# Install into sys.modules so `import rclpy` etc. Just Works.
# --------------------------------------------------------------------------
def install() -> None:
    """Register the mock packages in sys.modules. Call this BEFORE
    importing safety_controller_pkg.safety_node."""

    # rclpy package
    rclpy_mod = types.ModuleType("rclpy")
    rclpy_mod.init = init
    rclpy_mod.shutdown = shutdown
    rclpy_mod.spin = spin
    rclpy_mod.ok = ok

    # rclpy.node submodule
    rclpy_node_mod = types.ModuleType("rclpy.node")
    rclpy_node_mod.Node = Node
    rclpy_mod.node = rclpy_node_mod

    # rclpy.parameter submodule
    rclpy_parameter_mod = types.ModuleType("rclpy.parameter")
    rclpy_parameter_mod.Parameter = Parameter
    rclpy_mod.parameter = rclpy_parameter_mod

    # sensor_msgs package + .msg submodule
    sensor_msgs_mod = types.ModuleType("sensor_msgs")
    sensor_msgs_msg_mod = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg_mod.LaserScan = LaserScan
    sensor_msgs_mod.msg = sensor_msgs_msg_mod

    # nav_msgs package + .msg submodule
    nav_msgs_mod = types.ModuleType("nav_msgs")
    nav_msgs_msg_mod = types.ModuleType("nav_msgs.msg")
    nav_msgs_msg_mod.Odometry = Odometry
    nav_msgs_mod.msg = nav_msgs_msg_mod

    # std_msgs package + .msg submodule
    std_msgs_mod = types.ModuleType("std_msgs")
    std_msgs_msg_mod = types.ModuleType("std_msgs.msg")
    std_msgs_msg_mod.Bool = Bool
    std_msgs_msg_mod.Float32 = Float32
    std_msgs_mod.msg = std_msgs_msg_mod

    # ackermann_msgs package + .msg submodule
    ackermann_msgs_mod = types.ModuleType("ackermann_msgs")
    ackermann_msgs_msg_mod = types.ModuleType("ackermann_msgs.msg")
    ackermann_msgs_msg_mod.AckermannDriveStamped = AckermannDriveStamped
    ackermann_msgs_mod.msg = ackermann_msgs_msg_mod

    sys.modules["rclpy"] = rclpy_mod
    sys.modules["rclpy.node"] = rclpy_node_mod
    sys.modules["rclpy.parameter"] = rclpy_parameter_mod
    sys.modules["sensor_msgs"] = sensor_msgs_mod
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg_mod
    sys.modules["nav_msgs"] = nav_msgs_mod
    sys.modules["nav_msgs.msg"] = nav_msgs_msg_mod
    sys.modules["std_msgs"] = std_msgs_mod
    sys.modules["std_msgs.msg"] = std_msgs_msg_mod
    sys.modules["ackermann_msgs"] = ackermann_msgs_mod
    sys.modules["ackermann_msgs.msg"] = ackermann_msgs_msg_mod


def pump_timers(now_seconds: float) -> None:
    """Called by the sim loop each frame to fire any due timer callbacks."""
    for t in TIMERS:
        t.tick(now_seconds)
