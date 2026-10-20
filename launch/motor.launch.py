import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)
    overrides = {}

    yaml_path = LaunchConfiguration("yaml_path").perform(context).strip()
    if yaml_path:
        overrides["yaml_path"] = yaml_path
        module_key = LaunchConfiguration("module_key").perform(context).strip()
        if not module_key:
            overrides["module_key"] = Path(yaml_path).stem

    module_key = LaunchConfiguration("module_key").perform(context).strip()
    if module_key:
        overrides["module_key"] = module_key

    can_channel = LaunchConfiguration("can_channel").perform(context).strip()
    if can_channel:
        overrides["can_channel"] = can_channel

    for name in ("control_enabled", "auto_enable", "auto_configure"):
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            overrides[name] = value == "true"

    control_mode = LaunchConfiguration("control_mode").perform(context).strip()
    if control_mode:
        overrides["control_mode"] = control_mode

    for name in ("feedback_rate", "status_rate", "status_timeout"):
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            overrides[name] = float(value)

    parameters = [params_file]
    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package="motor_ros",
            executable="motor_node",
            name="motor_node",
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("motor_ros")
    default_params = os.path.join(share, "config", "motor_ros.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="ROS namespace (e.g. head, piper_h). Empty = root.",
            ),
            DeclareLaunchArgument(
                "yaml_path",
                default_value="",
                description="Motor YAML path (empty = use params_file)",
            ),
            DeclareLaunchArgument(
                "can_channel",
                default_value="",
                description="Override CAN channel (empty = use params_file / motor YAML)",
            ),
            DeclareLaunchArgument(
                "module_key",
                default_value="",
                description="YAML module key (empty = infer from yaml_path stem)",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="motor_ros parameter file",
            ),
            DeclareLaunchArgument(
                "control_enabled",
                default_value="",
                choices=["", "true", "false"],
                description="Control gate (empty = use params_file)",
            ),
            DeclareLaunchArgument(
                "control_mode",
                default_value="",
                choices=["", "pos", "mit"],
                description="Command mode (empty = use params_file)",
            ),
            DeclareLaunchArgument(
                "auto_enable",
                default_value="",
                choices=["", "true", "false"],
                description="Enable motors at startup (empty = use params_file)",
            ),
            DeclareLaunchArgument(
                "auto_configure",
                default_value="",
                choices=["", "true", "false"],
                description="Configure motors at startup (empty = use params_file)",
            ),
            DeclareLaunchArgument(
                "feedback_rate",
                default_value="",
                description="Joint feedback rate in Hz (empty = use params_file)",
            ),
            DeclareLaunchArgument(
                "status_rate",
                default_value="",
                description="Diagnostic rate in Hz (empty = use params_file)",
            ),
            DeclareLaunchArgument(
                "status_timeout",
                default_value="",
                description="Low-speed feedback timeout in seconds (empty = use params_file)",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
