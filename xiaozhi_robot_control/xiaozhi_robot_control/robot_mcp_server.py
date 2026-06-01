#!/usr/bin/env python3
"""A small stdio MCP server that publishes safe Diablo robot commands.

This file intentionally avoids third-party MCP packages so it can run on the
robot with only ROS2 Python packages installed. It implements the small subset
of JSON-RPC methods needed by MCP clients such as xiaozhi mcp_pipe.
"""

import argparse
import hashlib
import hmac
import json
import os
import socket
import sys
import threading
import time
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple

import rclpy
from motion_msgs.msg import MotionCtrl, RobotStatus
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


JsonDict = Dict[str, Any]


def _env_float(name: str, default: float) -> float:
    value = os.environ.get(name)
    if value is None:
        return default
    try:
        return float(value)
    except ValueError:
        return default


def _env_int(name: str, default: int) -> int:
    value = os.environ.get(name)
    if value is None:
        return default
    try:
        return int(value)
    except ValueError:
        return default


def _env_bool(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in ("1", "true", "yes", "on")


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _env_csv(name: str) -> List[str]:
    value = os.environ.get(name, "")
    return [item.strip() for item in value.split(",") if item.strip()]


def _json_for_signature(payload: JsonDict) -> bytes:
    return json.dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _relay_signature(key: str, payload: JsonDict) -> str:
    return hmac.new(key.encode("utf-8"), _json_for_signature(payload), hashlib.sha256).hexdigest()


def _parse_udp_targets(value: str) -> Dict[str, Tuple[str, int]]:
    targets: Dict[str, Tuple[str, int]] = {}
    for item in [part.strip() for part in value.split(",") if part.strip()]:
        if "=" not in item:
            continue
        label, address = [part.strip() for part in item.split("=", 1)]
        if not label or ":" not in address:
            continue
        host, port_text = address.rsplit(":", 1)
        try:
            port = int(port_text)
        except ValueError:
            continue
        targets[label] = (host.strip(), port)
    return targets


class LocalClusterTarget:
    """Cluster target that executes against the current robot process."""

    def __init__(self, robot: "DiabloRobotBridge", label: str, domain_id: str) -> None:
        self.robot = robot
        self.label = label
        self.domain_id = domain_id

    def warm(self) -> JsonDict:
        return self.ready(timeout_s=0.0)

    def ready(self, timeout_s: float) -> JsonDict:
        del timeout_s
        return {
            "ok": True,
            "robot_name": self.robot.robot_name,
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "transport": "local",
        }

    def call_tool(self, name: str, arguments: JsonDict, timeout_s: float) -> JsonDict:
        del timeout_s
        return {
            "ok": True,
            "robot_name": self.robot.robot_name,
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "transport": "local",
            "tool_result": self.robot._call_canonical_tool(name, arguments or {}),
        }

    def shutdown(self) -> None:
        return


class MissingClusterTarget:
    """Placeholder used when a remote robot has no UDP relay target configured."""

    def __init__(self, label: str, domain_id: str) -> None:
        self.label = label
        self.domain_id = domain_id

    def warm(self) -> JsonDict:
        raise RuntimeError(self._message())

    def ready(self, timeout_s: float) -> JsonDict:
        del timeout_s
        raise RuntimeError(self._message())

    def call_tool(self, name: str, arguments: JsonDict, timeout_s: float) -> JsonDict:
        del name, arguments, timeout_s
        raise RuntimeError(self._message())

    def shutdown(self) -> None:
        return

    def _message(self) -> str:
        return (
            "missing UDP relay target for %s; set DIABLO_CLUSTER_UDP_TARGETS="
            "robot2=IP:8765,robot3=IP:8765 on the dispatcher"
        ) % self.label


class UdpClusterTarget:
    """Cluster target that asks a remote robot to publish locally over UDP."""

    def __init__(self, label: str, domain_id: str, host: str, port: int, key: str) -> None:
        self.label = label
        self.domain_id = domain_id
        self.host = host
        self.port = port
        self.key = key
        self._lock = threading.Lock()
        self._request_id = 0

    def warm(self) -> JsonDict:
        timeout_s = _env_float("DIABLO_CLUSTER_UDP_READY_TIMEOUT_SEC", 2.0)
        return self._request("ping", {}, timeout_s=timeout_s)

    def ready(self, timeout_s: float) -> JsonDict:
        return self._request("ping", {}, timeout_s=max(0.2, timeout_s))

    def call_tool(self, name: str, arguments: JsonDict, timeout_s: float) -> JsonDict:
        return self._request(
            "call_tool",
            {"name": name, "arguments": arguments or {}},
            timeout_s=timeout_s,
        )

    def shutdown(self) -> None:
        return

    def _request(self, method: str, params: JsonDict, timeout_s: float) -> JsonDict:
        if not self.key:
            raise RuntimeError("DIABLO_CLUSTER_RELAY_KEY is required for UDP relay")

        with self._lock:
            self._request_id += 1
            request_id = self._request_id

        payload: JsonDict = {
            "id": request_id,
            "method": method,
            "params": params or {},
            "sender": os.environ.get("DIABLO_ROBOT_NAME", ""),
            "target": self.label,
            "ts": round(time.time(), 3),
        }
        payload["sig"] = _relay_signature(self.key, payload)
        encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(timeout_s)
            sock.sendto(encoded, (self.host, self.port))
            while True:
                data, _addr = sock.recvfrom(65535)
                response = json.loads(data.decode("utf-8"))
                if response.get("id") != request_id:
                    continue
                if "error" in response:
                    raise RuntimeError(str(response["error"]))
                return response.get("result") or {}


class ClusterToolDispatcher:
    """Fan out cluster tool calls to local or UDP relay targets."""

    TOOL_MAP = {
        "robot_cluster_stop": "robot_stop",
        "robot_cluster_move_forward": "robot_move_forward",
        "robot_cluster_move_backward": "robot_move_backward",
        "robot_cluster_turn_left": "robot_turn_left",
        "robot_cluster_turn_right": "robot_turn_right",
        "robot_cluster_raise_body": "robot_raise_body",
        "robot_cluster_lower_body": "robot_lower_body",
        "robot_cluster_pitch_up": "robot_pitch_up",
        "robot_cluster_pitch_down": "robot_pitch_down",
        "robot_cluster_roll_left": "robot_roll_left",
        "robot_cluster_roll_right": "robot_roll_right",
        "robot_cluster_reset_body_pose": "robot_reset_body_pose",
        "robot_cluster_get_status": "robot_get_status",
        "robot_cluster_stand_up": "robot_stand_up",
        "robot_cluster_stand_down": "robot_stand_down",
    }

    def __init__(self, robot: "DiabloRobotBridge", domains: List[str], labels: List[str]) -> None:
        self.robot = robot
        udp_targets = _parse_udp_targets(os.environ.get("DIABLO_CLUSTER_UDP_TARGETS", ""))
        relay_key = os.environ.get("DIABLO_CLUSTER_RELAY_KEY", os.environ.get("DIABLO_UDP_RELAY_KEY", ""))
        local_domain = os.environ.get("ROS_DOMAIN_ID", "")
        self.targets: List[Any] = []
        for index, domain_id in enumerate(domains):
            label = labels[index] if index < len(labels) else "robot%d" % (index + 1)
            if label == robot.robot_name or domain_id == local_domain:
                self.targets.append(LocalClusterTarget(robot, label, domain_id))
                continue
            target = udp_targets.get(label)
            if target is None:
                self.targets.append(MissingClusterTarget(label, domain_id))
                continue
            host, port = target
            self.targets.append(UdpClusterTarget(label, domain_id, host, port, relay_key))

    @classmethod
    def from_env(cls, robot: "DiabloRobotBridge") -> Optional["ClusterToolDispatcher"]:
        if not _env_bool("DIABLO_ENABLE_CLUSTER_TOOLS", False):
            return None
        domains = _env_csv("DIABLO_CLUSTER_ROS_DOMAIN_IDS") or [os.environ.get("ROS_DOMAIN_ID", "5")]
        labels = _env_csv("DIABLO_CLUSTER_ROBOT_NAMES")
        dispatcher = cls(robot, domains, labels)
        if _env_bool("DIABLO_CLUSTER_PRESTART", True):
            dispatcher.prestart()
        return dispatcher

    def prestart(self) -> None:
        errors: Dict[str, str] = {}
        lock = threading.Lock()

        def target_thread(target: Any) -> None:
            try:
                target.warm()
            except Exception as exc:
                with lock:
                    errors[target.label] = str(exc)

        threads = [threading.Thread(target=target_thread, args=(target,)) for target in self.targets]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        if errors:
            self.robot._log("cluster prestart warnings: %s" % json.dumps(errors, ensure_ascii=False))

    def tool_definitions(self) -> Iterable[JsonDict]:
        tools = [
            {
                "name": "robot_cluster_stop",
                "description": "Stop all configured robots immediately.",
                "inputSchema": self.robot._empty_schema(),
            },
            {
                "name": "robot_cluster_move_forward",
                "description": "Move all configured robots forward for a bounded duration, then stop automatically.",
                "inputSchema": self.robot._speed_duration_schema("Forward speed in meters per second."),
            },
            {
                "name": "robot_cluster_move_backward",
                "description": "Move all configured robots backward for a bounded duration, then stop automatically.",
                "inputSchema": self.robot._speed_duration_schema("Backward speed in meters per second."),
            },
            {
                "name": "robot_cluster_turn_left",
                "description": "Turn all configured robots left for a bounded duration, then stop automatically.",
                "inputSchema": self.robot._turn_duration_schema("Left turn angular speed in radians per second."),
            },
            {
                "name": "robot_cluster_turn_right",
                "description": "Turn all configured robots right for a bounded duration, then stop automatically.",
                "inputSchema": self.robot._turn_duration_schema("Right turn angular speed in radians per second."),
            },
            {
                "name": "robot_cluster_raise_body",
                "description": "Raise all configured robot bodies briefly, then stop automatically.",
                "inputSchema": self.robot._axis_duration_schema(
                    "Positive vertical command value.",
                    self.robot.default_vertical_speed,
                    self.robot.max_vertical_speed,
                ),
            },
            {
                "name": "robot_cluster_lower_body",
                "description": "Lower all configured robot bodies briefly, then stop automatically.",
                "inputSchema": self.robot._axis_duration_schema(
                    "Positive vertical command value. Each robot receives it as a negative up command.",
                    self.robot.default_vertical_speed,
                    self.robot.max_vertical_speed,
                ),
            },
            {
                "name": "robot_cluster_pitch_up",
                "description": "Tilt all configured robot bodies pitch upward briefly, then stop automatically.",
                "inputSchema": self.robot._axis_duration_schema(
                    "Positive pitch command value.",
                    self.robot.default_pitch,
                    self.robot.max_pitch,
                ),
            },
            {
                "name": "robot_cluster_pitch_down",
                "description": "Tilt all configured robot bodies pitch downward briefly, then stop automatically.",
                "inputSchema": self.robot._axis_duration_schema(
                    "Positive pitch command value. Each robot receives it as a negative pitch command.",
                    self.robot.default_pitch,
                    self.robot.max_pitch,
                ),
            },
            {
                "name": "robot_cluster_roll_left",
                "description": "Lean all configured robot bodies left briefly, then stop automatically.",
                "inputSchema": self.robot._axis_duration_schema(
                    "Positive roll command value. Each robot receives it as a negative roll command.",
                    self.robot.default_roll,
                    self.robot.max_roll,
                ),
            },
            {
                "name": "robot_cluster_roll_right",
                "description": "Lean all configured robot bodies right briefly, then stop automatically.",
                "inputSchema": self.robot._axis_duration_schema(
                    "Positive roll command value.",
                    self.robot.default_roll,
                    self.robot.max_roll,
                ),
            },
            {
                "name": "robot_cluster_reset_body_pose",
                "description": "Reset motion and body pose commands on all configured robots.",
                "inputSchema": self.robot._empty_schema(),
            },
            {
                "name": "robot_cluster_get_status",
                "description": "Read latest status and battery data from all configured robots.",
                "inputSchema": self.robot._empty_schema(),
            },
        ]
        if self.robot.enable_posture_tools:
            tools.extend(
                [
                    {
                        "name": "robot_cluster_stand_up",
                        "description": "Send stand-up command to all configured robots.",
                        "inputSchema": self.robot._empty_schema(),
                    },
                    {
                        "name": "robot_cluster_stand_down",
                        "description": "Send stand-down command to all configured robots.",
                        "inputSchema": self.robot._empty_schema(),
                    },
                ]
            )
        return tools

    def can_handle(self, name: str) -> bool:
        if name in ("robot_cluster_stand_up", "robot_cluster_stand_down"):
            return self.robot.enable_posture_tools
        return name in self.TOOL_MAP

    def call_tool(self, name: str, arguments: JsonDict) -> JsonDict:
        target_tool = self.TOOL_MAP.get(name)
        if target_tool is None or not self.can_handle(name):
            raise ValueError("unknown cluster tool: %s" % name)

        arguments = arguments or {}
        if self._requires_ready(target_tool) and _env_bool("DIABLO_CLUSTER_REQUIRE_ALL_READY", True):
            ready_results, ready_errors = self._prepare_targets()
            if ready_errors:
                return {
                    "ok": False,
                    "action": name,
                    "target_tool": target_tool,
                    "robot_count": len(self.targets),
                    "aborted": True,
                    "reason": "not all cluster targets are ready",
                    "ready_results": ready_results,
                    "errors": ready_errors,
                }

        timeout_s = self._tool_timeout_s(arguments)
        retry_count = max(1, _env_int("DIABLO_CLUSTER_CALL_RETRY_COUNT", 2))
        results: Dict[str, JsonDict] = {}
        errors: Dict[str, str] = {}
        lock = threading.Lock()

        def target_thread(target: Any) -> None:
            last_error = ""
            for attempt in range(retry_count):
                try:
                    result = target.call_tool(target_tool, arguments or {}, timeout_s=timeout_s)
                    with lock:
                        results[target.label] = result
                        errors.pop(target.label, None)
                    return
                except Exception as exc:
                    last_error = str(exc)
                    if attempt + 1 < retry_count:
                        time.sleep(0.1)
            with lock:
                errors[target.label] = last_error or "cluster target failed"

        threads = [threading.Thread(target=target_thread, args=(target,)) for target in self.targets]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        return {
            "ok": not errors and len(results) == len(self.targets),
            "action": name,
            "target_tool": target_tool,
            "robot_count": len(self.targets),
            "results": results,
            "errors": errors,
        }

    def shutdown(self) -> None:
        for target in self.targets:
            target.shutdown()

    def _prepare_targets(self) -> Tuple[Dict[str, JsonDict], Dict[str, str]]:
        retry_count = max(
            1,
            _env_int(
                "DIABLO_CLUSTER_READY_RETRY_COUNT",
                _env_int("DIABLO_CLUSTER_CALL_RETRY_COUNT", 2),
            ),
        )
        timeout_s = _env_float("DIABLO_CLUSTER_READY_TIMEOUT_SEC", self._default_ready_timeout_s())
        results: Dict[str, JsonDict] = {}
        errors: Dict[str, str] = {}
        lock = threading.Lock()

        def target_thread(target: Any) -> None:
            last_error = ""
            for attempt in range(retry_count):
                try:
                    result = target.ready(timeout_s=timeout_s)
                    with lock:
                        results[target.label] = result
                        errors.pop(target.label, None)
                    return
                except Exception as exc:
                    last_error = str(exc)
                    if attempt + 1 < retry_count:
                        time.sleep(0.2)
            with lock:
                errors[target.label] = last_error or "cluster target is not ready"

        threads = [threading.Thread(target=target_thread, args=(target,)) for target in self.targets]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        return results, errors

    def _requires_ready(self, target_tool: str) -> bool:
        return target_tool not in ("robot_get_status", "robot_stop")

    def _tool_timeout_s(self, arguments: JsonDict) -> float:
        duration_ms = int(arguments.get("duration_ms", self.robot.default_duration_ms))
        duration_s = max(0.0, duration_ms / 1000.0)
        extra_s = max(5.0, _env_float("DIABLO_CLUSTER_WORKER_TIMEOUT_EXTRA_SEC", 8.0))
        return duration_s + extra_s

    def _default_ready_timeout_s(self) -> float:
        return max(1.0, _env_float("DIABLO_CLUSTER_UDP_READY_TIMEOUT_SEC", 2.0))


class DiabloRobotBridge:
    """Translate high-level robot actions into /diablo/MotionCmd messages."""

    def __init__(self) -> None:
        self.motion_topic = os.environ.get("DIABLO_MOTION_TOPIC", "diablo/MotionCmd")
        self.status_topic = os.environ.get("DIABLO_STATUS_TOPIC", "diablo/sensor/Body_state")
        self.battery_topic = os.environ.get("DIABLO_BATTERY_TOPIC", "diablo/sensor/Battery")
        self.robot_name = os.environ.get("DIABLO_ROBOT_NAME", "robot2").strip() or "robot2"
        self.tool_prefix = os.environ.get("DIABLO_TOOL_PREFIX", self.robot_name + "_").strip()

        self.max_linear_speed = _env_float("DIABLO_MAX_LINEAR_SPEED", 0.5)
        self.max_turn_speed = _env_float("DIABLO_MAX_TURN_SPEED", 0.8)
        self.default_linear_speed = _env_float("DIABLO_DEFAULT_LINEAR_SPEED", self.max_linear_speed)
        self.default_turn_speed = _env_float("DIABLO_DEFAULT_TURN_SPEED", min(0.6, self.max_turn_speed))
        self.max_duration_ms = _env_int("DIABLO_MAX_DURATION_MS", 2000)
        self.min_duration_ms = _env_int("DIABLO_MIN_DURATION_MS", 1000)
        self.default_duration_ms = _env_int("DIABLO_DEFAULT_DURATION_MS", 1200)
        self.default_up = _env_float("DIABLO_DEFAULT_UP", 0.0)
        self.max_vertical_speed = _env_float("DIABLO_MAX_VERTICAL_SPEED", 1.0)
        self.default_vertical_speed = _env_float(
            "DIABLO_DEFAULT_VERTICAL_SPEED",
            min(0.5, self.max_vertical_speed),
        )
        self.stand_up_height = _env_float("DIABLO_STAND_UP_HEIGHT", min(1.0, self.max_vertical_speed))
        self.stand_up_height_publish_ms = _env_int("DIABLO_STAND_UP_HEIGHT_PUBLISH_MS", 1200)
        self.max_pitch = _env_float("DIABLO_MAX_PITCH", 0.5)
        self.default_pitch = _env_float("DIABLO_DEFAULT_PITCH", min(0.5, self.max_pitch))
        self.max_roll = _env_float("DIABLO_MAX_ROLL", 0.1)
        self.default_roll = _env_float("DIABLO_DEFAULT_ROLL", min(0.1, self.max_roll))
        self.action_reset_ms = _env_int("DIABLO_ACTION_RESET_MS", 300)
        self.command_period_ms = _env_int("DIABLO_COMMAND_PERIOD_MS", 20)
        self.stop_repeat_count = _env_int("DIABLO_STOP_REPEAT_COUNT", 30)
        self.wait_for_subscriber_ms = _env_int("DIABLO_WAIT_FOR_SUBSCRIBER_MS", 2000)
        self.discovery_settle_ms = _env_int("DIABLO_DISCOVERY_SETTLE_MS", 1000)
        self.enable_posture_tools = _env_bool("DIABLO_ENABLE_POSTURE_TOOLS", False)

        rclpy.init(args=None)
        node_name = os.environ.get("DIABLO_MCP_NODE_NAME", "xiaozhi_robot_mcp_bridge")
        self.node: Node = rclpy.create_node(node_name)
        self.publisher = self.node.create_publisher(MotionCtrl, self.motion_topic, 10)
        self.node.create_subscription(RobotStatus, self.status_topic, self._status_callback, 10)
        self.node.create_subscription(BatteryState, self.battery_topic, self._battery_callback, 10)

        self._lock = threading.Lock()
        self._latest_status: Optional[RobotStatus] = None
        self._latest_status_time = 0.0
        self._latest_battery: Optional[BatteryState] = None
        self._latest_battery_time = 0.0
        self._motion_subscriber_seen = False
        self._udp_relay_socket: Optional[socket.socket] = None
        self._udp_relay_stop = threading.Event()
        self._udp_relay_thread: Optional[threading.Thread] = None

        self._spin_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self._spin_thread.start()
        self._start_udp_relay()
        self.cluster_tools = ClusterToolDispatcher.from_env(self)
        self._log(
                "robot MCP bridge ready: node=%s robot_name=%s tool_prefix=%s motion_topic=%s max_linear=%.2f max_turn=%.2f default_linear=%.2f min_duration_ms=%d max_duration_ms=%d default_up=%.2f default_vertical=%.2f stand_up_height=%.2f default_pitch=%.2f default_roll=%.2f ros_domain_id=%s rmw=%s fastdds_profile=%s cluster_tools=%s udp_relay=%s"
            % (
                node_name,
                self.robot_name,
                self.tool_prefix,
                self.motion_topic,
                self.max_linear_speed,
                self.max_turn_speed,
                self.default_linear_speed,
                self.min_duration_ms,
                self.max_duration_ms,
                self.default_up,
                self.default_vertical_speed,
                self.stand_up_height,
                self.default_pitch,
                self.default_roll,
                os.environ.get("ROS_DOMAIN_ID", ""),
                os.environ.get("RMW_IMPLEMENTATION", ""),
                os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", ""),
                "enabled" if self.cluster_tools is not None else "disabled",
                "enabled" if self._udp_relay_socket is not None else "disabled",
            )
        )

    def _start_udp_relay(self) -> None:
        if not _env_bool("DIABLO_UDP_RELAY_LISTEN", False):
            return
        key = os.environ.get("DIABLO_UDP_RELAY_KEY", os.environ.get("DIABLO_CLUSTER_RELAY_KEY", ""))
        if not key:
            self._log("UDP relay disabled: DIABLO_UDP_RELAY_KEY is not set")
            return
        host = os.environ.get("DIABLO_UDP_RELAY_HOST", "0.0.0.0")
        port = _env_int("DIABLO_UDP_RELAY_PORT", 8765)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((host, port))
            sock.settimeout(0.5)
        except Exception:
            sock.close()
            raise
        self._udp_relay_socket = sock
        self._udp_relay_thread = threading.Thread(target=self._udp_relay_loop, daemon=True)
        self._udp_relay_thread.start()
        self._log("UDP relay listening on %s:%d" % (host, port))

    def _udp_relay_loop(self) -> None:
        assert self._udp_relay_socket is not None
        while not self._udp_relay_stop.is_set():
            try:
                data, addr = self._udp_relay_socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            self._handle_udp_relay_request(data, addr)

    def _handle_udp_relay_request(self, data: bytes, addr: Tuple[str, int]) -> None:
        request_id: Any = None
        response: JsonDict
        try:
            request = json.loads(data.decode("utf-8"))
            request_id = request.get("id")
            self._verify_udp_relay_request(request)
            method = request.get("method")
            params = request.get("params") or {}
            if method in ("ping", "warm", "ready"):
                response = {
                    "id": request_id,
                    "result": {
                        "ok": True,
                        "robot_name": self.robot_name,
                        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
                        "transport": "udp",
                    },
                }
            elif method == "call_tool":
                name = self._canonical_tool_name(str(params.get("name", "")))
                if name.startswith("robot_cluster_"):
                    raise ValueError("UDP relay only accepts single-robot tools")
                result = self._call_canonical_tool(name, params.get("arguments") or {})
                response = {
                    "id": request_id,
                    "result": {
                        "ok": True,
                        "robot_name": self.robot_name,
                        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
                        "transport": "udp",
                        "tool_result": result,
                    },
                }
            else:
                raise ValueError("unknown UDP relay method: %s" % method)
        except Exception as exc:
            response = {"id": request_id, "error": str(exc)}
        try:
            encoded = json.dumps(response, ensure_ascii=False).encode("utf-8")
            if self._udp_relay_socket is not None:
                self._udp_relay_socket.sendto(encoded, addr)
        except Exception as exc:
            self._log("UDP relay response failed: %s" % exc)

    def _verify_udp_relay_request(self, request: JsonDict) -> None:
        key = os.environ.get("DIABLO_UDP_RELAY_KEY", os.environ.get("DIABLO_CLUSTER_RELAY_KEY", ""))
        if not key:
            raise RuntimeError("DIABLO_UDP_RELAY_KEY is not set")
        signature = str(request.get("sig", ""))
        payload = dict(request)
        payload.pop("sig", None)
        expected = _relay_signature(key, payload)
        if not hmac.compare_digest(signature, expected):
            raise RuntimeError("invalid UDP relay signature")

    def _stop_udp_relay(self) -> None:
        self._udp_relay_stop.set()
        if self._udp_relay_socket is not None:
            try:
                self._udp_relay_socket.close()
            except OSError:
                pass
            self._udp_relay_socket = None
        if self._udp_relay_thread is not None:
            self._udp_relay_thread.join(timeout=1.0)
            self._udp_relay_thread = None

    def shutdown(self) -> None:
        self._stop_udp_relay()
        try:
            self.stop()
        except Exception as exc:  # pragma: no cover - best effort shutdown
            self._log("stop during shutdown failed: %s" % exc)
        try:
            if self.cluster_tools is not None:
                self.cluster_tools.shutdown()
        except Exception as exc:  # pragma: no cover - best effort shutdown
            self._log("cluster shutdown failed: %s" % exc)
        try:
            self.node.destroy_node()
        finally:
            rclpy.shutdown()

    def tool_definitions(self) -> Iterable[JsonDict]:
        tools = [
            {
                "name": self._public_tool_name("robot_stop"),
                "description": "Stop %s immediately by publishing a zero motion command." % self.robot_name,
                "inputSchema": {
                    "type": "object",
                    "properties": {},
                    "additionalProperties": False,
                },
            },
            {
                "name": self._public_tool_name("robot_move_forward"),
                "description": "Move %s forward for a short bounded duration, then stop automatically." % self.robot_name,
                "inputSchema": self._speed_duration_schema("Forward speed in meters per second."),
            },
            {
                "name": self._public_tool_name("robot_move_backward"),
                "description": "Move %s backward for a short bounded duration, then stop automatically." % self.robot_name,
                "inputSchema": self._speed_duration_schema("Backward speed in meters per second."),
            },
            {
                "name": self._public_tool_name("robot_turn_left"),
                "description": "Turn %s left for a short bounded duration, then stop automatically." % self.robot_name,
                "inputSchema": self._turn_duration_schema("Left turn angular speed in radians per second."),
            },
            {
                "name": self._public_tool_name("robot_turn_right"),
                "description": "Turn %s right for a short bounded duration, then stop automatically." % self.robot_name,
                "inputSchema": self._turn_duration_schema("Right turn angular speed in radians per second."),
            },
            {
                "name": self._public_tool_name("robot_raise_body"),
                "description": "Raise %s body briefly using the same vertical control as the teleop up keys, then stop automatically." % self.robot_name,
                "inputSchema": self._axis_duration_schema(
                    "Positive vertical command value. Matches the teleop up control.",
                    self.default_vertical_speed,
                    self.max_vertical_speed,
                ),
            },
            {
                "name": self._public_tool_name("robot_lower_body"),
                "description": "Lower %s body briefly using the same vertical control as the teleop down key, then stop automatically." % self.robot_name,
                "inputSchema": self._axis_duration_schema(
                    "Positive vertical command value. The server sends it as a negative up command.",
                    self.default_vertical_speed,
                    self.max_vertical_speed,
                ),
            },
            {
                "name": self._public_tool_name("robot_pitch_up"),
                "description": "Tilt %s body pitch upward briefly, then stop automatically." % self.robot_name,
                "inputSchema": self._axis_duration_schema(
                    "Positive pitch command value.",
                    self.default_pitch,
                    self.max_pitch,
                ),
            },
            {
                "name": self._public_tool_name("robot_pitch_down"),
                "description": "Tilt %s body pitch downward briefly, then stop automatically." % self.robot_name,
                "inputSchema": self._axis_duration_schema(
                    "Positive pitch command value. The server sends it as a negative pitch command.",
                    self.default_pitch,
                    self.max_pitch,
                ),
            },
            {
                "name": self._public_tool_name("robot_roll_left"),
                "description": "Lean %s body left briefly, then stop automatically." % self.robot_name,
                "inputSchema": self._axis_duration_schema(
                    "Positive roll command value. The server sends it as a negative roll command.",
                    self.default_roll,
                    self.max_roll,
                ),
            },
            {
                "name": self._public_tool_name("robot_roll_right"),
                "description": "Lean %s body right briefly, then stop automatically." % self.robot_name,
                "inputSchema": self._axis_duration_schema(
                    "Positive roll command value.",
                    self.default_roll,
                    self.max_roll,
                ),
            },
            {
                "name": self._public_tool_name("robot_reset_body_pose"),
                "description": "Reset %s motion, vertical, pitch, roll, and leg split commands to neutral." % self.robot_name,
                "inputSchema": self._empty_schema(),
            },
            {
                "name": self._public_tool_name("robot_get_status"),
                "description": "Read the latest %s status and battery data received by ROS2." % self.robot_name,
                "inputSchema": {
                    "type": "object",
                    "properties": {},
                    "additionalProperties": False,
                },
            },
        ]
        if self.enable_posture_tools:
            tools.extend(
                [
                    {
                        "name": self._public_tool_name("robot_stand_up"),
                        "description": "Send a one-shot stand-up command to %s, then command maximum body height." % self.robot_name,
                        "inputSchema": self._empty_schema(),
                    },
                    {
                        "name": self._public_tool_name("robot_stand_down"),
                        "description": "Send a one-shot stand-down command to %s, then reset to zero motion." % self.robot_name,
                        "inputSchema": self._empty_schema(),
                    },
                ]
            )
        if self.cluster_tools is not None:
            tools.extend(self.cluster_tools.tool_definitions())
        return tools

    def call_tool(self, name: str, arguments: JsonDict) -> JsonDict:
        self._log("tool call: %s arguments=%s" % (name, json.dumps(arguments or {}, ensure_ascii=False)))
        if self.cluster_tools is not None and self.cluster_tools.can_handle(name):
            return self.cluster_tools.call_tool(name, arguments or {})

        name = self._canonical_tool_name(name)
        return self._call_canonical_tool(name, arguments or {})

    def _call_canonical_tool(self, name: str, arguments: JsonDict) -> JsonDict:
        handlers: Dict[str, Callable[[JsonDict], JsonDict]] = {
            "robot_stop": lambda _: self.stop(),
            "robot_move_forward": self.move_forward,
            "robot_move_backward": self.move_backward,
            "robot_turn_left": self.turn_left,
            "robot_turn_right": self.turn_right,
            "robot_raise_body": self.raise_body,
            "robot_lower_body": self.lower_body,
            "robot_pitch_up": self.pitch_up,
            "robot_pitch_down": self.pitch_down,
            "robot_roll_left": self.roll_left,
            "robot_roll_right": self.roll_right,
            "robot_reset_body_pose": lambda _: self.reset_body_pose(),
            "robot_get_status": lambda _: self.get_status(),
        }
        if self.enable_posture_tools:
            handlers["robot_stand_up"] = lambda _: self.stand(True)
            handlers["robot_stand_down"] = lambda _: self.stand(False)

        handler = handlers.get(name)
        if handler is None:
            raise ValueError("unknown tool: %s" % name)
        return handler(arguments or {})

    def _public_tool_name(self, canonical_name: str) -> str:
        if canonical_name.startswith("robot_") and self.tool_prefix:
            return self.tool_prefix + canonical_name[len("robot_") :]
        return canonical_name

    def _canonical_tool_name(self, public_name: str) -> str:
        if self.tool_prefix and public_name.startswith(self.tool_prefix):
            return "robot_" + public_name[len(self.tool_prefix) :]
        return public_name

    def move_forward(self, arguments: JsonDict) -> JsonDict:
        speed = self._bounded_speed(arguments, "speed", self.default_linear_speed, self.max_linear_speed)
        duration_ms = self._bounded_duration(arguments)
        return self._move(forward=speed, left=0.0, duration_ms=duration_ms)

    def move_backward(self, arguments: JsonDict) -> JsonDict:
        speed = self._bounded_speed(arguments, "speed", self.default_linear_speed, self.max_linear_speed)
        duration_ms = self._bounded_duration(arguments)
        return self._move(forward=-speed, left=0.0, duration_ms=duration_ms)

    def turn_left(self, arguments: JsonDict) -> JsonDict:
        speed = self._bounded_speed(arguments, "speed", self.default_turn_speed, self.max_turn_speed)
        duration_ms = self._bounded_duration(arguments)
        return self._move(forward=0.0, left=speed, duration_ms=duration_ms)

    def turn_right(self, arguments: JsonDict) -> JsonDict:
        speed = self._bounded_speed(arguments, "speed", self.default_turn_speed, self.max_turn_speed)
        duration_ms = self._bounded_duration(arguments)
        return self._move(forward=0.0, left=-speed, duration_ms=duration_ms)

    def raise_body(self, arguments: JsonDict) -> JsonDict:
        value = self._bounded_value(arguments, self.default_vertical_speed, self.max_vertical_speed)
        duration_ms = self._bounded_duration(arguments)
        return self._move(up=value, duration_ms=duration_ms)

    def lower_body(self, arguments: JsonDict) -> JsonDict:
        value = self._bounded_value(arguments, self.default_vertical_speed, self.max_vertical_speed)
        duration_ms = self._bounded_duration(arguments)
        return self._move(up=-value, duration_ms=duration_ms)

    def pitch_up(self, arguments: JsonDict) -> JsonDict:
        value = self._bounded_value(arguments, self.default_pitch, self.max_pitch)
        duration_ms = self._bounded_duration(arguments)
        return self._move(pitch=value, duration_ms=duration_ms)

    def pitch_down(self, arguments: JsonDict) -> JsonDict:
        value = self._bounded_value(arguments, self.default_pitch, self.max_pitch)
        duration_ms = self._bounded_duration(arguments)
        return self._move(pitch=-value, duration_ms=duration_ms)

    def roll_left(self, arguments: JsonDict) -> JsonDict:
        value = self._bounded_value(arguments, self.default_roll, self.max_roll)
        duration_ms = self._bounded_duration(arguments)
        return self._move(roll=-value, duration_ms=duration_ms)

    def roll_right(self, arguments: JsonDict) -> JsonDict:
        value = self._bounded_value(arguments, self.default_roll, self.max_roll)
        duration_ms = self._bounded_duration(arguments)
        return self._move(roll=value, duration_ms=duration_ms)

    def reset_body_pose(self) -> JsonDict:
        self._publish_stop_repeated()
        return {"ok": True, "action": "reset_body_pose"}

    def stop(self) -> JsonDict:
        self._publish_stop_repeated()
        return {"ok": True, "action": "stop"}

    def stand(self, stand_up: bool) -> JsonDict:
        msg = MotionCtrl()
        msg.mode_mark = True
        msg.mode.stand_mode = bool(stand_up)
        self._publish(msg)
        time.sleep(max(0, self.action_reset_ms) / 1000.0)

        if stand_up:
            stand_up_height = _clamp(abs(self.stand_up_height), 0.0, self.max_vertical_speed)
            publish_count = self._publish_motion_for(
                up=stand_up_height,
                duration_ms=max(0, self.stand_up_height_publish_ms),
                description="stand up max height",
            )
            return {
                "ok": True,
                "action": "stand_up",
                "up": round(stand_up_height, 3),
                "height_publish_count": publish_count,
            }

        self.stop()
        return {"ok": True, "action": "stand_down"}

    def get_status(self) -> JsonDict:
        now = time.time()
        with self._lock:
            status = self._latest_status
            status_age = now - self._latest_status_time if status is not None else None
            battery = self._latest_battery
            battery_age = now - self._latest_battery_time if battery is not None else None

        result: JsonDict = {"ok": True}
        if status is None:
            result["status"] = None
        else:
            result["status"] = {
                "ctrl_mode": int(status.ctrl_mode_msg),
                "robot_mode": int(status.robot_mode_msg),
                "error": int(status.error_msg),
                "warning": int(status.warning_msg),
                "age_seconds": round(status_age or 0.0, 3),
            }
        if battery is None:
            result["battery"] = None
        else:
            result["battery"] = {
                "voltage": float(battery.voltage),
                "current": float(battery.current),
                "percentage": float(battery.percentage),
                "age_seconds": round(battery_age or 0.0, 3),
            }
        return result

    def _move(
        self,
        forward: float = 0.0,
        left: float = 0.0,
        up: Optional[float] = None,
        roll: float = 0.0,
        pitch: float = 0.0,
        leg_split: float = 0.0,
        duration_ms: Optional[int] = None,
    ) -> JsonDict:
        if duration_ms is None:
            duration_ms = min(self.default_duration_ms, self.max_duration_ms)
        motion_up = self.default_up if up is None else up
        self._log(
            "publish motion: forward=%.3f left=%.3f up=%.3f roll=%.3f pitch=%.3f duration_ms=%d"
            % (forward, left, motion_up, roll, pitch, duration_ms)
        )
        period_s = max(0.02, self.command_period_ms / 1000.0)
        self._wait_for_motion_subscriber()
        deadline = time.monotonic() + (duration_ms / 1000.0)
        publish_count = 0
        try:
            while True:
                self._publish_motion(
                    forward=forward,
                    left=left,
                    up=motion_up,
                    roll=roll,
                    pitch=pitch,
                    leg_split=leg_split,
                )
                publish_count += 1
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(period_s, remaining))
        finally:
            self._log("auto stop after motion after %d motion publishes" % publish_count)
            self._publish_stop_repeated(period_s=period_s)
        return {
            "ok": True,
            "action": "move",
            "forward": round(forward, 3),
            "left": round(left, 3),
            "up": round(motion_up, 3),
            "roll": round(roll, 3),
            "pitch": round(pitch, 3),
            "duration_ms": duration_ms,
        }

    def _publish_motion(
        self,
        forward: float,
        left: float,
        up: Optional[float] = None,
        roll: float = 0.0,
        pitch: float = 0.0,
        leg_split: float = 0.0,
        log: bool = True,
    ) -> None:
        motion_up = self.default_up if up is None else up
        msg = MotionCtrl()
        msg.mode_mark = False
        msg.value.forward = float(forward)
        msg.value.left = float(left)
        msg.value.up = float(motion_up)
        msg.value.roll = float(roll)
        msg.value.pitch = float(pitch)
        msg.value.leg_split = float(leg_split)
        self._publish(msg)
        if log:
            self._log(
                "published /%s forward=%.3f left=%.3f up=%.3f roll=%.3f pitch=%.3f"
                % (self.motion_topic, forward, left, motion_up, roll, pitch)
            )

    def _publish_motion_for(
        self,
        forward: float = 0.0,
        left: float = 0.0,
        up: Optional[float] = None,
        roll: float = 0.0,
        pitch: float = 0.0,
        leg_split: float = 0.0,
        duration_ms: int = 0,
        description: str = "hold motion",
    ) -> int:
        period_s = max(0.02, self.command_period_ms / 1000.0)
        deadline = time.monotonic() + max(0, duration_ms) / 1000.0
        publish_count = 0
        while True:
            self._publish_motion(
                forward=forward,
                left=left,
                up=up,
                roll=roll,
                pitch=pitch,
                leg_split=leg_split,
                log=(publish_count == 0),
            )
            publish_count += 1
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(period_s, remaining))
        self._log("%s published %d motion commands" % (description, publish_count))
        return publish_count

    def _publish_stop_repeated(self, period_s: Optional[float] = None) -> None:
        interval = period_s if period_s is not None else max(0.02, self.command_period_ms / 1000.0)
        repeat_count = max(1, self.stop_repeat_count)
        for index in range(repeat_count):
            self._publish_motion(forward=0.0, left=0.0, log=False)
            if index + 1 < repeat_count:
                time.sleep(interval)
        self._log("published stop x%d" % repeat_count)

    def _publish(self, msg: MotionCtrl) -> None:
        self._wait_for_motion_subscriber()
        with self._lock:
            self.publisher.publish(msg)

    def _wait_for_motion_subscriber(self) -> None:
        if not _env_bool("DIABLO_CLUSTER_REQUIRE_SUBSCRIBER", True):
            return
        if self.wait_for_subscriber_ms <= 0:
            return
        if self._motion_subscriber_seen and self.publisher.get_subscription_count() > 0:
            return

        deadline = time.monotonic() + self.wait_for_subscriber_ms / 1000.0
        while self.publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.05)

        if self.publisher.get_subscription_count() > 0:
            if self.discovery_settle_ms > 0:
                time.sleep(self.discovery_settle_ms / 1000.0)
            self._motion_subscriber_seen = True
            self._log("motion subscriber detected on /%s" % self.motion_topic)
        else:
            self._log(
                "publish without detected motion subscriber after %d ms"
                % self.wait_for_subscriber_ms
            )

    def _status_callback(self, msg: RobotStatus) -> None:
        with self._lock:
            self._latest_status = msg
            self._latest_status_time = time.time()

    def _battery_callback(self, msg: BatteryState) -> None:
        with self._lock:
            self._latest_battery = msg
            self._latest_battery_time = time.time()

    def _bounded_speed(self, arguments: JsonDict, name: str, default: float, maximum: float) -> float:
        speed = float(arguments.get(name, default))
        return _clamp(abs(speed), 0.0, maximum)

    def _bounded_value(self, arguments: JsonDict, default: float, maximum: float) -> float:
        for name in ("value", "speed", "amount"):
            if name in arguments:
                return _clamp(abs(float(arguments[name])), 0.0, maximum)
        return _clamp(abs(default), 0.0, maximum)

    def _bounded_duration(self, arguments: JsonDict) -> int:
        duration_ms = int(arguments.get("duration_ms", min(self.default_duration_ms, self.max_duration_ms)))
        return int(_clamp(duration_ms, self.min_duration_ms, self.max_duration_ms))

    def _speed_duration_schema(self, speed_description: str) -> JsonDict:
        return {
            "type": "object",
            "properties": {
                "speed": {
                    "type": "number",
                    "description": speed_description,
                    "minimum": 0,
                    "maximum": self.max_linear_speed,
                    "default": min(self.default_linear_speed, self.max_linear_speed),
                },
                "duration_ms": {
                    "type": "integer",
                    "description": "Duration in milliseconds. The server always stops the robot afterwards.",
                    "minimum": self.min_duration_ms,
                    "maximum": self.max_duration_ms,
                    "default": min(self.default_duration_ms, self.max_duration_ms),
                },
            },
            "additionalProperties": False,
        }

    def _turn_duration_schema(self, speed_description: str) -> JsonDict:
        schema = self._speed_duration_schema(speed_description)
        schema["properties"]["speed"]["maximum"] = self.max_turn_speed
        schema["properties"]["speed"]["default"] = min(self.default_turn_speed, self.max_turn_speed)
        return schema

    def _axis_duration_schema(self, value_description: str, default: float, maximum: float) -> JsonDict:
        return {
            "type": "object",
            "properties": {
                "value": {
                    "type": "number",
                    "description": value_description,
                    "minimum": 0,
                    "maximum": maximum,
                    "default": min(default, maximum),
                },
                "duration_ms": {
                    "type": "integer",
                    "description": "Duration in milliseconds. The server always resets the command afterwards.",
                    "minimum": self.min_duration_ms,
                    "maximum": self.max_duration_ms,
                    "default": min(self.default_duration_ms, self.max_duration_ms),
                },
            },
            "additionalProperties": False,
        }

    def _empty_schema(self) -> JsonDict:
        return {
            "type": "object",
            "properties": {},
            "additionalProperties": False,
        }

    def _log(self, message: str) -> None:
        print("[xiaozhi_robot_control] %s" % message, file=sys.stderr, flush=True)


class JsonRpcMcpServer:
    """Minimal JSON-RPC MCP server over newline-delimited stdio."""

    def __init__(self, robot: DiabloRobotBridge) -> None:
        self.robot = robot

    def serve_forever(self) -> None:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                request = json.loads(line)
                response = self._handle_message(request)
            except Exception as exc:
                response = self._error(None, -32700, "parse or dispatch error: %s" % exc)
            if response is not None:
                self._write(response)

    def _handle_message(self, request: JsonDict) -> Optional[JsonDict]:
        if not isinstance(request, dict):
            return self._error(None, -32600, "invalid JSON-RPC request")

        request_id = request.get("id")
        method = request.get("method")
        params = request.get("params") or {}
        is_notification = "id" not in request

        if method == "initialize":
            return self._result(
                request_id,
                {
                    "protocolVersion": params.get("protocolVersion", "2024-11-05"),
                    "capabilities": {"tools": {"listChanged": False}},
                    "serverInfo": {
                        "name": "diablo-robot-mcp",
                        "version": "0.1.0",
                    },
                },
            )
        if method == "tools/list":
            return self._result(request_id, {"tools": list(self.robot.tool_definitions())})
        if method == "tools/call":
            return self._handle_tool_call(request_id, params)
        if method == "ping":
            return self._result(request_id, {})
        if method in ("resources/list", "prompts/list"):
            key = "resources" if method == "resources/list" else "prompts"
            return self._result(request_id, {key: []})
        if isinstance(method, str) and method.startswith("notifications/"):
            return None

        if is_notification:
            return None
        return self._error(request_id, -32601, "method not found: %s" % method)

    def _handle_tool_call(self, request_id: Any, params: JsonDict) -> JsonDict:
        name = params.get("name")
        arguments = params.get("arguments") or {}
        try:
            result = self.robot.call_tool(str(name), arguments)
            text = json.dumps(result, ensure_ascii=False)
            return self._result(
                request_id,
                {
                    "content": [{"type": "text", "text": text}],
                    "isError": False,
                },
            )
        except Exception as exc:
            self.robot._log("tool error: %s" % exc)
            return self._result(
                request_id,
                {
                    "content": [{"type": "text", "text": str(exc)}],
                    "isError": True,
                },
            )

    def _result(self, request_id: Any, result: JsonDict) -> JsonDict:
        return {"jsonrpc": "2.0", "id": request_id, "result": result}

    def _error(self, request_id: Any, code: int, message: str) -> JsonDict:
        return {
            "jsonrpc": "2.0",
            "id": request_id,
            "error": {"code": code, "message": message},
        }

    def _write(self, response: JsonDict) -> None:
        sys.stdout.write(json.dumps(response, ensure_ascii=False) + "\n")
        sys.stdout.flush()


def main() -> None:
    parser = argparse.ArgumentParser(description="Diablo robot MCP stdio server")
    parser.add_argument(
        "--list-tools",
        action="store_true",
        help="Print tool definitions as JSON and exit after initializing ROS.",
    )
    args = parser.parse_args()

    robot = DiabloRobotBridge()
    try:
        if args.list_tools:
            print(json.dumps({"tools": list(robot.tool_definitions())}, ensure_ascii=False, indent=2))
            return
        JsonRpcMcpServer(robot).serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        robot.shutdown()


if __name__ == "__main__":
    main()
