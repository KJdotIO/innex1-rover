# Copyright 2026 Leicester Lunabotics Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Browser Gamepad API bridge to ROS ``cmd_vel``."""

from __future__ import annotations

import json
import secrets
import ssl
import threading
import time
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node

from lunabot_interfaces.msg import DrivetrainStatus, DrivetrainTelemetry


@dataclass(frozen=True)
class GamepadCommand:
    """A validated browser gamepad command."""

    linear_x: float
    angular_z: float
    enabled: bool


@dataclass(frozen=True)
class SpeedLimits:
    """Runtime speed limits accepted from the operator page."""

    max_linear_mps: float
    max_angular_radps: float


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a floating point value to an inclusive range."""
    return max(lower, min(upper, value))


def parse_command(payload: dict[str, Any]) -> GamepadCommand:
    """Convert untrusted browser JSON into a bounded command."""
    return GamepadCommand(
        linear_x=clamp(float(payload.get("linear_x", 0.0)), -1.0, 1.0),
        angular_z=clamp(float(payload.get("angular_z", 0.0)), -1.0, 1.0),
        enabled=bool(payload.get("enabled", False)),
    )


def parse_speed_limits(
    payload: dict[str, Any],
    *,
    current_linear_mps: float,
    current_angular_radps: float,
) -> SpeedLimits:
    """Convert operator speed settings into bounded bridge limits."""
    return SpeedLimits(
        max_linear_mps=clamp(
            float(payload.get("max_linear_mps", current_linear_mps)),
            0.01,
            0.60,
        ),
        max_angular_radps=clamp(
            float(payload.get("max_angular_radps", current_angular_radps)),
            0.05,
            1.60,
        ),
    )


def command_to_twist(
    command: GamepadCommand,
    *,
    max_linear_mps: float,
    max_angular_radps: float,
) -> Twist:
    """Scale a validated gamepad command into a ROS Twist."""
    msg = Twist()
    if not command.enabled:
        return msg
    msg.linear.x = command.linear_x * max_linear_mps
    msg.angular.z = command.angular_z * max_angular_radps
    return msg


def all_interface_bind_requires_tls(
    bind_host: str,
    tls_cert_file: str,
    tls_key_file: str,
) -> bool:
    """Return true when a public bind address lacks a complete TLS pair."""
    return bind_host in {"0.0.0.0", "::"} and not (tls_cert_file and tls_key_file)


class WebGamepadBridge(Node):
    """Serve a controller page and publish fresh browser gamepad commands."""

    def __init__(self) -> None:
        """Initialise ROS publishers and the HTTP server."""
        super().__init__("web_gamepad_bridge")
        self.declare_parameter("bind_host", "127.0.0.1")
        self.declare_parameter("port", 8080)
        self.declare_parameter("tls_cert_file", "")
        self.declare_parameter("tls_key_file", "")
        self.declare_parameter("auth_token", "")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_safe")
        self.declare_parameter("max_linear_mps", 0.30)
        self.declare_parameter("max_angular_radps", 0.80)
        self.declare_parameter("command_timeout_s", 0.35)
        self.declare_parameter("publish_hz", 20.0)

        self._bind_host = str(self.get_parameter("bind_host").value)
        self._port = int(self.get_parameter("port").value)
        self._tls_cert_file = str(self.get_parameter("tls_cert_file").value)
        self._tls_key_file = str(self.get_parameter("tls_key_file").value)
        configured_token = str(self.get_parameter("auth_token").value)
        self._auth_token = configured_token or secrets.token_urlsafe(24)
        self._max_linear = float(self.get_parameter("max_linear_mps").value)
        self._max_angular = float(self.get_parameter("max_angular_radps").value)
        self._timeout_s = float(self.get_parameter("command_timeout_s").value)
        publish_hz = float(self.get_parameter("publish_hz").value)
        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)

        if self._port <= 0:
            raise ValueError(f"port must be positive: {self._port}")
        if self._timeout_s <= 0.0:
            raise ValueError(f"command_timeout_s must be positive: {self._timeout_s}")
        if publish_hz <= 0.0:
            raise ValueError(f"publish_hz must be positive: {publish_hz}")
        if all_interface_bind_requires_tls(
            self._bind_host,
            self._tls_cert_file,
            self._tls_key_file,
        ):
            raise ValueError(
                "Refusing to expose web gamepad bridge on all interfaces without TLS"
            )

        self._lock = threading.Lock()
        self._last_command = GamepadCommand(0.0, 0.0, False)
        self._last_command_time = 0.0
        self._drivetrain_status: dict[str, Any] = {}
        self._drivetrain_telemetry: dict[str, Any] = {}
        self._publisher = self.create_publisher(Twist, cmd_topic, 10)
        self._status_sub = self.create_subscription(
            DrivetrainStatus,
            "/drivetrain/status",
            self._status_callback,
            10,
        )
        self._telemetry_sub = self.create_subscription(
            DrivetrainTelemetry,
            "/drivetrain/telemetry",
            self._telemetry_callback,
            10,
        )
        self._timer = self.create_timer(1.0 / publish_hz, self._publish_latest)
        self._server = self._start_http_server()

        self.get_logger().info(
            "Web gamepad bridge ready: "
            f"{self._scheme()}://{self._bind_host}:{self._port} -> {cmd_topic}"
        )

    def receive_command(self, command: GamepadCommand) -> None:
        """Store the latest command from the browser."""
        with self._lock:
            self._last_command = command
            self._last_command_time = time.monotonic()

    def update_speed_limits(self, limits: SpeedLimits) -> None:
        """Apply operator speed limits without restarting the bridge."""
        with self._lock:
            self._max_linear = limits.max_linear_mps
            self._max_angular = limits.max_angular_radps

    def state_snapshot(self) -> dict[str, Any]:
        """Return current bridge and drivetrain state for the operator page."""
        with self._lock:
            command_age = time.monotonic() - self._last_command_time
            return {
                "limits": {
                    "max_linear_mps": self._max_linear,
                    "max_angular_radps": self._max_angular,
                    "command_timeout_s": self._timeout_s,
                },
                "command": {
                    "linear_x": self._last_command.linear_x,
                    "angular_z": self._last_command.angular_z,
                    "enabled": self._last_command.enabled,
                    "age_s": command_age,
                    "fresh": command_age <= self._timeout_s,
                },
                "drivetrain_status": dict(self._drivetrain_status),
                "drivetrain_telemetry": dict(self._drivetrain_telemetry),
            }

    def _publish_latest(self) -> None:
        """Publish the latest command, or zero if it has gone stale."""
        with self._lock:
            age = time.monotonic() - self._last_command_time
            command = self._last_command
            max_linear = self._max_linear
            max_angular = self._max_angular

        if age > self._timeout_s:
            command = GamepadCommand(0.0, 0.0, False)

        self._publisher.publish(
            command_to_twist(
                command,
                max_linear_mps=max_linear,
                max_angular_radps=max_angular,
            )
        )

    def _status_callback(self, msg: DrivetrainStatus) -> None:
        """Store the latest drivetrain status for the operator page."""
        with self._lock:
            self._drivetrain_status = {
                "state": int(msg.state),
                "fault_code": int(msg.fault_code),
                "estop_active": bool(msg.estop_active),
                "motion_inhibited": bool(msg.motion_inhibited),
                "controller_online": list(msg.controller_online),
            }

    def _telemetry_callback(self, msg: DrivetrainTelemetry) -> None:
        """Store the latest drivetrain telemetry for the operator page."""
        with self._lock:
            self._drivetrain_telemetry = {
                "wheel_velocity_rps": [
                    float(value) for value in msg.wheel_velocity_rps
                ],
                "encoder_ticks": [int(value) for value in msg.encoder_ticks],
                "controller_online": list(msg.controller_online),
                "estop_active": bool(msg.estop_active),
                "motion_inhibited": bool(msg.motion_inhibited),
                "fault_code": int(msg.fault_code),
            }

    def _start_http_server(self) -> ThreadingHTTPServer:
        """Start the HTTP server used by the browser controller page."""
        node = self
        web_root = Path(get_package_share_directory("lunabot_teleop")) / "web"

        class Handler(BaseHTTPRequestHandler):
            """HTTP handler bound to this bridge instance."""

            def log_message(self, fmt, *args):
                node.get_logger().debug(fmt % args)

            def do_GET(self) -> None:
                if self.path == "/api/state":
                    self._send_json(node.state_snapshot())
                    return
                if self.path not in {"/", "/index.html"}:
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                self._send_index(web_root / "index.html")

            def do_HEAD(self) -> None:
                if self.path not in {"/", "/index.html"}:
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()

            def do_POST(self) -> None:
                if not self._has_valid_operator_token():
                    self.send_error(HTTPStatus.FORBIDDEN)
                    return
                if self.path == "/api/config":
                    self._handle_config()
                    return
                if self.path != "/api/command":
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    body = self.rfile.read(length)
                    payload = json.loads(body.decode("utf-8"))
                    node.receive_command(parse_command(payload))
                except (ValueError, TypeError, json.JSONDecodeError) as exc:
                    self.send_error(HTTPStatus.BAD_REQUEST, explain=str(exc))
                    return
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()

            def _handle_config(self) -> None:
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    body = self.rfile.read(length)
                    payload = json.loads(body.decode("utf-8"))
                    snapshot = node.state_snapshot()["limits"]
                    limits = parse_speed_limits(
                        payload,
                        current_linear_mps=float(snapshot["max_linear_mps"]),
                        current_angular_radps=float(snapshot["max_angular_radps"]),
                    )
                    node.update_speed_limits(limits)
                except (ValueError, TypeError, json.JSONDecodeError) as exc:
                    self.send_error(HTTPStatus.BAD_REQUEST, explain=str(exc))
                    return
                self._send_json(node.state_snapshot()["limits"])

            def _has_valid_operator_token(self) -> bool:
                token = self.headers.get("X-Lunabot-Operator-Token", "")
                return secrets.compare_digest(token, node._auth_token)

            def _send_index(self, path: Path) -> None:
                try:
                    content = path.read_text(encoding="utf-8")
                except OSError:
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                content = content.replace("__LUNABOT_AUTH_TOKEN__", node._auth_token)
                self._send_bytes(content.encode("utf-8"), "text/html")

            def _send_bytes(self, content: bytes, content_type: str) -> None:
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", f"{content_type}; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)

            def _send_json(self, payload: dict[str, Any]) -> None:
                content = json.dumps(payload).encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)

        server = ThreadingHTTPServer((self._bind_host, self._port), Handler)
        if self._tls_cert_file and self._tls_key_file:
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            context.load_cert_chain(
                certfile=self._tls_cert_file,
                keyfile=self._tls_key_file,
            )
            server.socket = context.wrap_socket(server.socket, server_side=True)
        thread = threading.Thread(
            target=server.serve_forever,
            name="web_gamepad_http",
            daemon=True,
        )
        thread.start()
        return server

    def _scheme(self) -> str:
        """Return the browser URL scheme currently served."""
        if self._tls_cert_file and self._tls_key_file:
            return "https"
        return "http"

    def destroy_node(self) -> None:
        """Stop HTTP service and publish one final zero command."""
        self._publisher.publish(Twist())
        self._server.shutdown()
        self._server.server_close()
        super().destroy_node()


def main(args=None) -> None:
    """Run the web gamepad bridge node."""
    rclpy.init(args=args)
    node = WebGamepadBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
