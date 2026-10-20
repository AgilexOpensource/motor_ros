#!/usr/bin/env python3
"""ROS 2 node bridging agx_motor_sdk ArmJointGroup to control/feedback topics."""

import math
import os
import time
from pathlib import Path
from typing import Dict, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from agx_motor_sdk.mechanism import ArmJointGroup

PACKAGE_NAME = "motor_ros"
DEFAULT_MOTOR_YAML = "default.yaml"
CONTROL_MODE_POS = "pos"
CONTROL_MODE_MIT = "mit"


def load_torque_limit_by_joint(config_path: Path, module_key: str) -> Dict[str, float]:
    """Read per-joint ``torque_limit`` [N·m]; joints without it are omitted."""
    with config_path.open(encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    module = data.get(module_key)
    if not isinstance(module, dict):
        return {}

    limits: Dict[str, float] = {}
    for entry in module.get("joints") or []:
        if not isinstance(entry, dict):
            continue
        joint_name = entry.get("joint_name")
        if not joint_name or "torque_limit" not in entry:
            continue
        limit = entry.get("torque_limit")
        if limit is not None:
            limits[str(joint_name)] = float(limit)
    return limits


def torque_limit_to_current_a(
    torque_limit: float, torque_k: float, torque_b: float
) -> float:
    """Inverse of SDK torque estimate: ``torque_nm = current_a × torque_k × torque_b``."""
    scale = float(torque_k) * float(torque_b)
    if not math.isfinite(scale) or scale <= 0.0:
        raise ValueError("torque_k × torque_b must be finite and > 0")
    limit = float(torque_limit)
    if not math.isfinite(limit) or limit < 0.0:
        raise ValueError("torque_limit must be finite and >= 0")
    current = limit / scale
    if not math.isfinite(current):
        raise ValueError("converted current limit must be finite")
    return current


def package_config_dir() -> Path:
    return Path(get_package_share_directory(PACKAGE_NAME)) / "config"


def resolve_yaml_path(yaml_path: str) -> Path:
    """Resolve motor YAML: empty → default.yaml; relative → share/config/."""
    raw = yaml_path.strip()
    if not raw:
        raw = os.environ.get("MOTOR_ROS_CONFIG", "").strip() or DEFAULT_MOTOR_YAML

    path = Path(raw)
    if path.is_absolute():
        if not path.is_file():
            raise FileNotFoundError("motor YAML not found: %s" % path)
        return path

    share_config = package_config_dir()
    candidates = [
        share_config / path,
        share_config / path.name,
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate

    raise FileNotFoundError(
        "motor YAML not found: %s (searched under %s)" % (raw, share_config)
    )


def resolve_module_key(module_key: str, config_path: Path) -> str:
    """Use explicit module_key, else infer from motor YAML filename stem."""
    key = str(module_key).strip()
    if key:
        return key
    stem = config_path.stem
    if stem:
        return stem
    raise ValueError("module_key is empty and cannot be inferred from yaml_path")


class MotorRosNode(Node):
    """Publish joint feedback and accept JointState commands on control/feedback topics."""

    def __init__(self) -> None:
        super().__init__("motor_node")

        startup_parameters = {
            "yaml_path": "",
            "module_key": "",
            "can_channel": "",
            "auto_configure": True,
            "auto_enable": True,
            "feedback_rate": 100.0,
            "status_rate": 2.0,
            "status_timeout": 2.0,
            "control_enabled": True,
        }
        for name, default in startup_parameters.items():
            self.declare_parameter(name, default, ParameterDescriptor(read_only=True))
        self.declare_parameter("control_mode", CONTROL_MODE_POS)

        yaml_path = str(self.get_parameter("yaml_path").value).strip()
        can_channel = str(self.get_parameter("can_channel").value).strip() or None
        self._auto_configure = bool(self.get_parameter("auto_configure").value)
        self._auto_enable = bool(self.get_parameter("auto_enable").value)
        feedback_rate = float(self.get_parameter("feedback_rate").value)
        if not math.isfinite(feedback_rate) or feedback_rate <= 0.0:
            raise ValueError("feedback_rate must be finite and > 0")
        status_rate = float(self.get_parameter("status_rate").value)
        self._status_timeout = float(self.get_parameter("status_timeout").value)
        if not math.isfinite(status_rate) or status_rate <= 0.0:
            raise ValueError("status_rate must be finite and > 0")
        if not math.isfinite(self._status_timeout) or self._status_timeout <= 0.0:
            raise ValueError("status_timeout must be finite and > 0")
        self._control_mode = self._parse_control_mode(
            self.get_parameter("control_mode").value
        )
        self._control_enabled = bool(self.get_parameter("control_enabled").value)
        self._control_gate_block_logged = False

        config_path = resolve_yaml_path(yaml_path)
        module_key = resolve_module_key(
            str(self.get_parameter("module_key").value), config_path
        )
        self._group = ArmJointGroup.from_yaml(str(config_path), module_key, can_channel)

        self._joint_by_name: Dict[str, int] = {
            name: self._group.index_of(name) for name in self._group.joint_names
        }
        self._joint_names = list(self._group.joint_names)

        try:
            if not self._group.connect():
                raise RuntimeError("failed to open CAN: %s" % self._group.can_channel)
            if self._auto_configure:
                if not self._group.configure():
                    raise RuntimeError("motor configure failed")
                self._apply_torque_limits(config_path, module_key)
            if self._auto_enable and not self._group.enable():
                raise RuntimeError("motor enable failed")
        except Exception:
            try:
                self._group.close()
            except Exception as exc:
                self.get_logger().warn("startup cleanup: %s" % exc)
            raise

        self._feedback_pub = self.create_publisher(JointState, "feedback/joint_states", 10)
        self._status_pub = self.create_publisher(DiagnosticArray, "feedback/motor_status", 10)
        self._control_sub = self.create_subscription(
            JointState, "control/joint_states", self._on_control, 10
        )
        self._enable_srv = self.create_service(SetBool, "enable", self._on_enable)
        self.create_service(SetBool, "control_enable", self._on_control_enable)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        period = 1.0 / feedback_rate
        self._feedback_timer = self.create_timer(period, self._publish_feedback)
        self._status_timer = self.create_timer(1.0 / status_rate, self._publish_status)

        self.get_logger().info(
            "motor_ros ready module=%s can=%s joints=[%s] mode=%s gate=%s"
            % (
                module_key,
                self._group.can_channel,
                ", ".join(self._joint_names),
                self._control_mode,
                self._control_enabled,
            )
        )

    def _apply_torque_limits(self, config_path: Path, module_key: str) -> None:
        limits = load_torque_limit_by_joint(config_path, module_key)
        if not limits:
            return

        for index in range(self._group.joint_count):
            joint_cfg = self._group.joint_config(index)
            limit = limits.get(joint_cfg.name)
            if limit is None:
                continue

            try:
                current_a = torque_limit_to_current_a(
                    limit, joint_cfg.torque_k, joint_cfg.torque_b
                )
            except ValueError as exc:
                raise RuntimeError(
                    "invalid torque scale for %s: %s" % (joint_cfg.name, exc)
                ) from exc

            motor = self._group.motor(index)
            if not motor.is_valid:
                self.get_logger().warn(
                    "skip current limit for %s: motor not valid" % joint_cfg.name
                )
                continue

            if not motor.set_current_limit(current_a):
                raise RuntimeError(
                    "set_current_limit failed for %s (%.3f A from %.3f N·m)"
                    % (joint_cfg.name, current_a, limit)
                )

            self.get_logger().info(
                "%s current_limit=%.3f A (torque_limit=%.3f)"
                % (joint_cfg.name, current_a, limit)
            )

    def _check_can_control(self) -> bool:
        if not self._control_enabled:
            if not self._control_gate_block_logged:
                self.get_logger().info(
                    "External control gate is closed, ignore control commands"
                )
                self._control_gate_block_logged = True
            return False
        self._control_gate_block_logged = False
        return True

    @staticmethod
    def _parse_control_mode(value) -> str:
        mode = str(value).strip().lower()
        if mode not in (CONTROL_MODE_POS, CONTROL_MODE_MIT):
            raise ValueError("control_mode must be '%s' or '%s'" % (
                CONTROL_MODE_POS, CONTROL_MODE_MIT
            ))
        return mode

    @staticmethod
    def _safe_get(values, index: int, default: float = 0.0) -> float:
        if index >= len(values):
            return default
        return float(values[index])

    def _on_set_parameters(self, params) -> SetParametersResult:
        control_mode = self._control_mode
        for param in params:
            if param.name != "control_mode":
                continue
            try:
                control_mode = self._parse_control_mode(param.value)
            except ValueError as exc:
                return SetParametersResult(successful=False, reason=str(exc))
        if control_mode != self._control_mode:
            self._control_mode = control_mode
            self.get_logger().info("control_mode -> %s" % control_mode)
        return SetParametersResult(successful=True)

    def _publish_status(self) -> None:
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        now_ns = time.time_ns()
        fault_fields = (
            "undervoltage", "motor_overtemp", "driver_overcurrent",
            "driver_overtemp", "collision_tripped", "driver_error", "stall_tripped",
        )
        numeric_fields = (
            "bus_voltage_v", "driver_temp_deg", "motor_temp_deg", "bus_current", "status_raw",
        )
        for index in range(self._group.joint_count):
            motor = self._group.motor(index)
            status = DiagnosticStatus()
            status.name = self._joint_names[index]
            status.hardware_id = "%s:%s" % (motor.can_channel, motor.node_id)
            status.level = DiagnosticStatus.STALE
            status.message = "low-speed feedback unavailable"
            values = {
                name: "unknown" for name in ("enabled",) + fault_fields + numeric_fields
            }
            values["control_mode"] = self._control_mode
            feedback = motor.get_low_speed_feedback()
            if feedback is not None:
                age = (now_ns - feedback.timestamp_ns) / 1e9
                if feedback.timestamp_ns > 0 and 0.0 <= age <= self._status_timeout:
                    values["enabled"] = str(feedback.status.enabled).lower()
                    faults = []
                    for name in fault_fields:
                        active = getattr(feedback.status, name)
                        values[name] = str(active).lower()
                        if active:
                            faults.append(name)
                    for name in numeric_fields:
                        values[name] = str(getattr(feedback, name))
                    status.level = DiagnosticStatus.OK
                    status.message = "enabled" if feedback.status.enabled else "disabled"
                    if faults:
                        status.level = DiagnosticStatus.ERROR
                        status.message = ", ".join(faults)
                else:
                    status.message = "low-speed feedback timestamp invalid or expired"
            status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
            msg.status.append(status)
        self._status_pub.publish(msg)

    def _publish_feedback(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []

        for index in range(self._group.joint_count):
            state = self._group.get_state(index)
            if state is None:
                continue
            msg.name.append(self._joint_names[index])
            msg.position.append(state.position_rad)
            msg.velocity.append(state.velocity_rad_s)
            msg.effort.append(state.torque_nm)

        if msg.name:
            self._feedback_pub.publish(msg)

    def _on_control(self, msg: JointState) -> None:
        if not self._check_can_control():
            return
        if not msg.name or not msg.position:
            self.get_logger().warn("control/joint_states ignored: empty name or position")
            return
        if len(msg.name) != len(msg.position):
            self.get_logger().warn("control/joint_states ignored: name/position size mismatch")
            return
        if len(set(msg.name)) != len(msg.name):
            self.get_logger().warn("control/joint_states ignored: duplicate joint names")
            return
        command_fields = [msg.position]
        if self._control_mode == CONTROL_MODE_MIT:
            for values in (msg.velocity, msg.effort):
                if values and len(values) != len(msg.name):
                    self.get_logger().warn(
                        "control/joint_states ignored: velocity/effort size mismatch"
                    )
                    return
                command_fields.append(values)
        if any(not math.isfinite(value) for values in command_fields for value in values):
            self.get_logger().warn("control/joint_states ignored: non-finite command")
            return

        for idx, name in enumerate(msg.name):
            index = self._joint_by_name.get(name)
            if index is None or index == ArmJointGroup.INVALID_JOINT_INDEX:
                self.get_logger().debug("skip unknown joint: %s" % name)
                continue

            position = self._safe_get(msg.position, idx)
            if self._control_mode == CONTROL_MODE_MIT:
                velocity = self._safe_get(msg.velocity, idx)
                torque = self._safe_get(msg.effort, idx)
                ok = self._group.set_mit(index, position, velocity, torque)
                if not ok:
                    self.get_logger().warn("set_mit failed for %s" % name)
            else:
                if not self._group.set_target_position(index, position):
                    self.get_logger().warn("set_target_position failed for %s" % name)

    def _on_control_enable(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        self._control_enabled = bool(request.data)
        self._control_gate_block_logged = False
        state = "opened" if request.data else "closed"
        response.success = True
        response.message = "External control gate %s" % state
        self.get_logger().info(response.message)
        return response

    def _on_enable(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        ok = self._group.enable() if request.data else self._group.disable()
        response.success = ok
        response.message = "enabled" if request.data else "disabled"
        if not ok:
            response.message += " (failed)"
        return response

    def destroy_node(self) -> bool:
        try:
            self._group.close()
        except Exception as exc:
            self.get_logger().warn("shutdown cleanup: %s" % exc)
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MotorRosNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
