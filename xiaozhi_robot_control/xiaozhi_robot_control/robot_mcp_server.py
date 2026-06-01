#!/usr/bin/env python3
"""A small stdio MCP server that publishes safe Diablo robot commands.

This file intentionally avoids third-party MCP packages so it can run on the
robot with only ROS2 Python packages installed. It implements the small subset
of JSON-RPC methods needed by MCP clients such as xiaozhi mcp_pipe.
"""

import argparse
import json
import os
import subprocess
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


def _subscriber_wait_timeout_s(extra_s: float = 3.0) -> float:
    wait_s = max(0.0, _env_int("DIABLO_WAIT_FOR_SUBSCRIBER_MS", 2000) / 1000.0)
    settle_s = max(0.0, _env_int("DIABLO_DISCOVERY_SETTLE_MS", 1000) / 1000.0)
    return max(5.0, wait_s + settle_s + extra_s)


class DomainWorkerClient:
    """Persistent helper process that publishes commands in one ROS domain."""

    def __init__(self, domain_id: str, label: str) -> None:
        self.domain_id = domain_id
        self.label = label
        self.process: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()
        self._request_id = 0

    def warm(self) -> JsonDict:
        with self._lock:
            timeout_s = _env_float("DIABLO_CLUSTER_WARM_TIMEOUT_SEC", _subscriber_wait_timeout_s(5.0))
            return self._request_with_restart_locked("warm", {}, timeout_s=timeout_s)

    def ready(self, timeout_s: float) -> JsonDict:
        with self._lock:
            return self._request_with_restart_locked("ready", {}, timeout_s=timeout_s)

    def call_tool(self, name: str, arguments: JsonDict, timeout_s: float) -> JsonDict:
        with self._lock:
            return self._request_with_restart_locked(
                "call_tool",
                {"name": name, "arguments": arguments or {}},
                timeout_s=timeout_s,
            )

    def shutdown(self) -> None:
        with self._lock:
            self._stop_locked(send_stop=True)

    def _request_with_restart_locked(self, method: str, params: JsonDict, timeout_s: float) -> JsonDict:
        attempts = max(1, _env_int("DIABLO_CLUSTER_WORKER_RESTART_ATTEMPTS", 2))
        last_error = ""
        for attempt in range(attempts):
            try:
                self._ensure_started_locked()
                return self._request_locked(method, params, timeout_s=timeout_s)
            except Exception as exc:
                last_error = str(exc)
                self._stop_locked(send_stop=False)
                if attempt + 1 >= attempts:
                    break
        raise RuntimeError(last_error or "worker request failed")

    def _ensure_started_locked(self) -> None:
        if self.process is not None and self.process.poll() is None:
            return

        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(self.domain_id)
        env["DIABLO_ENABLE_CLUSTER_TOOLS"] = "0"
        env["DIABLO_ROBOT_NAME"] = self.label
        env["DIABLO_MCP_NODE_NAME"] = "xiaozhi_cluster_worker_%s" % self._safe_label()

        command = [sys.executable, os.path.abspath(__file__), "--domain-worker"]
        self.process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            bufsize=1,
            env=env,
        )
        threading.Thread(target=self._stderr_loop, daemon=True).start()

    def _request_locked(self, method: str, params: JsonDict, timeout_s: float) -> JsonDict:
        if self.process is None or self.process.stdin is None or self.process.stdout is None:
            raise RuntimeError("domain worker is not running")
        if self.process.poll() is not None:
            raise RuntimeError("domain worker exited with code %s" % self.process.returncode)

        self._request_id += 1
        request_id = self._request_id
        self.process.stdin.write(
            json.dumps(
                {
                    "id": request_id,
                    "method": method,
                    "params": params,
                },
                ensure_ascii=False,
            )
            + "\n"
        )
        self.process.stdin.flush()

        response_box: Dict[str, Any] = {}

        def read_response() -> None:
            try:
                line = self.process.stdout.readline() if self.process is not None and self.process.stdout is not None else ""
                response_box["line"] = line
            except Exception as exc:
                response_box["error"] = exc

        reader = threading.Thread(target=read_response, daemon=True)
        reader.start()
        reader.join(timeout_s)
        if reader.is_alive():
            raise TimeoutError("domain worker request timed out after %.1fs" % timeout_s)
        if "error" in response_box:
            raise RuntimeError(str(response_box["error"]))

        line = response_box.get("line", "")
        if not line:
            raise RuntimeError("domain worker closed stdout")
        response = json.loads(line)
        if response.get("id") != request_id:
            raise RuntimeError("domain worker response id mismatch")
        if "error" in response:
            raise RuntimeError(response["error"])
        return response.get("result") or {}

    def _stop_locked(self, send_stop: bool) -> None:
        if self.process is None:
            return
        try:
            if send_stop and self.process.poll() is None:
                try:
                    self._request_locked(
                        "call_tool",
                        {"name": "robot_stop", "arguments": {}},
                        timeout_s=_subscriber_wait_timeout_s(5.0),
                    )
                except Exception:
                    pass
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=5)
        finally:
            self.process = None

    def _stderr_loop(self) -> None:
        if self.process is None or self.process.stderr is None:
            return
        for line in self.process.stderr:
            sys.stderr.write("[cluster:%s:%s] %s" % (self.label, self.domain_id, line))
            sys.stderr.flush()

    def _safe_label(self) -> str:
        return "".join(ch if ch.isalnum() else "_" for ch in self.label) or self.domain_id


class ClusterToolDispatcher:
    """Fan out cluster tool calls to persistent per-domain workers."""

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
        self.workers = [
            DomainWorkerClient(domain_id, labels[index] if index < len(labels) else "robot%d" % (index + 1))
            for index, domain_id in enumerate(domains)
        ]

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

        def worker_thread(worker: DomainWorkerClient) -> None:
            try:
                worker.warm()
            except Exception as exc:
                with lock:
                    errors[worker.label] = str(exc)

        threads = [threading.Thread(target=worker_thread, args=(worker,)) for worker in self.workers]
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
            ready_results, ready_errors = self._prepare_workers()
            if ready_errors:
                return {
                    "ok": False,
                    "action": name,
                    "target_tool": target_tool,
                    "robot_count": len(self.workers),
                    "aborted": True,
                    "reason": "not all cluster workers are ready",
                    "ready_results": ready_results,
                    "errors": ready_errors,
                }

        timeout_s = self._tool_timeout_s(arguments)
        retry_count = max(1, _env_int("DIABLO_CLUSTER_CALL_RETRY_COUNT", 2))
        results: Dict[str, JsonDict] = {}
        errors: Dict[str, str] = {}
        lock = threading.Lock()

        def worker_thread(worker: DomainWorkerClient) -> None:
            last_error = ""
            for attempt in range(retry_count):
                try:
                    result = worker.call_tool(target_tool, arguments or {}, timeout_s=timeout_s)
                    with lock:
                        results[worker.label] = result
                        errors.pop(worker.label, None)
                    return
                except Exception as exc:
                    last_error = str(exc)
                    if attempt + 1 < retry_count:
                        time.sleep(0.1)
            with lock:
                errors[worker.label] = last_error or "cluster worker failed"

        threads = [threading.Thread(target=worker_thread, args=(worker,)) for worker in self.workers]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        return {
            "ok": not errors and len(results) == len(self.workers),
            "action": name,
            "target_tool": target_tool,
            "robot_count": len(self.workers),
            "results": results,
            "errors": errors,
        }

    def shutdown(self) -> None:
        for worker in self.workers:
            worker.shutdown()

    def _prepare_workers(self) -> Tuple[Dict[str, JsonDict], Dict[str, str]]:
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

        def worker_thread(worker: DomainWorkerClient) -> None:
            last_error = ""
            for attempt in range(retry_count):
                try:
                    result = worker.ready(timeout_s=timeout_s)
                    with lock:
                        results[worker.label] = result
                        errors.pop(worker.label, None)
                    return
                except Exception as exc:
                    last_error = str(exc)
                    if attempt + 1 < retry_count:
                        time.sleep(0.2)
            with lock:
                errors[worker.label] = last_error or "cluster worker is not ready"

        threads = [threading.Thread(target=worker_thread, args=(worker,)) for worker in self.workers]
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
        subscriber_wait_s = self._default_ready_timeout_s()
        extra_s = max(5.0, _env_float("DIABLO_CLUSTER_WORKER_TIMEOUT_EXTRA_SEC", 8.0))
        return duration_s + subscriber_wait_s + extra_s

    def _default_ready_timeout_s(self) -> float:
        wait_s = max(0.0, self.robot.wait_for_subscriber_ms / 1000.0)
        settle_s = max(0.0, self.robot.discovery_settle_ms / 1000.0)
        return max(5.0, wait_s + settle_s + 3.0)


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

        self._spin_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self._spin_thread.start()
        self.cluster_tools = ClusterToolDispatcher.from_env(self)
        self._log(
            "robot MCP bridge ready: node=%s robot_name=%s tool_prefix=%s motion_topic=%s max_linear=%.2f max_turn=%.2f default_linear=%.2f min_duration_ms=%d max_duration_ms=%d default_up=%.2f default_vertical=%.2f stand_up_height=%.2f default_pitch=%.2f default_roll=%.2f ros_domain_id=%s rmw=%s fastdds_profile=%s cluster_tools=%s"
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
            )
        )

    def shutdown(self) -> None:
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


def serve_domain_worker() -> None:
    """Serve direct JSON commands for one persistent ROS domain worker."""

    robot = DiabloRobotBridge()
    try:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            response: JsonDict
            request_id: Any = None
            try:
                request = json.loads(line)
                request_id = request.get("id")
                method = request.get("method")
                params = request.get("params") or {}
                if method in ("warm", "ready"):
                    robot._wait_for_motion_subscriber()
                    subscriber_count = robot.publisher.get_subscription_count()
                    if (
                        method == "ready"
                        and _env_bool("DIABLO_CLUSTER_REQUIRE_SUBSCRIBER", True)
                        and subscriber_count == 0
                    ):
                        raise RuntimeError(
                            "no motion subscriber on /%s in ROS_DOMAIN_ID=%s"
                            % (robot.motion_topic, os.environ.get("ROS_DOMAIN_ID", ""))
                        )
                    response = {
                        "id": request_id,
                        "result": {
                            "ok": True,
                            "robot_name": robot.robot_name,
                            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
                            "subscriber_count": subscriber_count,
                        },
                    }
                elif method == "call_tool":
                    name = str(params.get("name"))
                    if name != "robot_get_status" and _env_bool("DIABLO_CLUSTER_REQUIRE_SUBSCRIBER", True):
                        robot._wait_for_motion_subscriber()
                        if robot.publisher.get_subscription_count() == 0:
                            raise RuntimeError(
                                "no motion subscriber on /%s in ROS_DOMAIN_ID=%s"
                                % (robot.motion_topic, os.environ.get("ROS_DOMAIN_ID", ""))
                            )
                    result = robot.call_tool(name, params.get("arguments") or {})
                    response = {
                        "id": request_id,
                        "result": {
                            "ok": True,
                            "robot_name": robot.robot_name,
                            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
                            "tool_result": result,
                        },
                    }
                else:
                    response = {"id": request_id, "error": "unknown worker method: %s" % method}
            except Exception as exc:
                response = {"id": request_id, "error": str(exc)}
            sys.stdout.write(json.dumps(response, ensure_ascii=False) + "\n")
            sys.stdout.flush()
    finally:
        robot.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description="Diablo robot MCP stdio server")
    parser.add_argument(
        "--list-tools",
        action="store_true",
        help="Print tool definitions as JSON and exit after initializing ROS.",
    )
    parser.add_argument(
        "--domain-worker",
        action="store_true",
        help="Run as an internal persistent worker for one ROS domain.",
    )
    args = parser.parse_args()

    if args.domain_worker:
        serve_domain_worker()
        return

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
