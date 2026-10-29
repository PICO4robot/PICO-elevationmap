"""ROS2 bringup：LiDAR 驱动 + D435i + elevation map 可视化。

用法（在仓库根目录，先 source ROS2 和对应驱动工作区）：

    ros2 launch ./launch/bringup.launch.py \
        lidar:=livox lidar_topic:=/livox/lidar lidar_msg_type:=custom \
        config:=$PWD/scripts/tools/configs/livox_mid360.yaml

Airy 示例：

    ros2 launch ./launch/bringup.launch.py \
        lidar:=airy lidar_topic:=/rslidar_points lidar_msg_type:=pointcloud2 \
        config:=$PWD/scripts/tools/configs/realsense_lidar.yaml

只跑 D435i：`lidar:=none`。
"""

from __future__ import annotations

import shutil
from pathlib import Path

from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription

_REPO_ROOT = Path(__file__).resolve().parent.parent
_VENV_PYTHON = _REPO_ROOT / ".venv-ros" / "bin" / "python"
_DEFAULT_CONFIG = _REPO_ROOT / "scripts" / "tools" / "configs" / "livox_mid360.yaml"
_AIRY_CONFIG = _REPO_ROOT / "scripts" / "tools" / "configs" / "airy_rslidar.yaml"


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ("1", "true", "yes", "on")


def _require_python(python: str) -> None:
    """Fail fast with an actionable error when the mapper interpreter is missing.

    ``python`` remains a launch argument so users can point at another ROS-visible
    interpreter; only the existence/resolution check is added here.
    """
    path = Path(python).expanduser()
    if path.is_absolute() or path.parent != Path("."):
        if not path.is_file():
            raise RuntimeError(
                f"python executable not found: {python}. Create the ROS-visible venv first: "
                "`uv venv --system-site-packages .venv-ros` then "
                "`UV_PROJECT_ENVIRONMENT=.venv-ros uv sync --extra cuda --extra realsense`, "
                "or pass `python:=/path/to/python`."
            )
        return
    if shutil.which(python) is None:
        raise RuntimeError(
            f"python executable not found on PATH: {python}. Pass "
            "`python:=/path/to/ros-visible/python` or create `.venv-ros` as described "
            "in docs/development.md section 7."
        )


def _launch_setup(context: object) -> list:
    lidar = LaunchConfiguration("lidar").perform(context)
    lidar_topic = LaunchConfiguration("lidar_topic").perform(context)
    lidar_msg_type = LaunchConfiguration("lidar_msg_type").perform(context)
    config = LaunchConfiguration("config").perform(context)
    python = LaunchConfiguration("python").perform(context)
    frames = LaunchConfiguration("frames").perform(context)
    view = _as_bool(LaunchConfiguration("view").perform(context))
    voxel_view = _as_bool(LaunchConfiguration("voxel_view").perform(context))
    foxglove = _as_bool(LaunchConfiguration("foxglove").perform(context))
    delay = float(LaunchConfiguration("start_delay").perform(context))

    _require_python(python)

    actions: list = []
    if lidar == "livox":
        actions.append(
            ExecuteProcess(
                cmd=["ros2", "launch", "livox_ros_driver2", LaunchConfiguration("livox_launch").perform(context)],
                output="screen",
            )
        )
    elif lidar == "airy":
        actions.append(
            Node(
                package="rslidar_sdk",
                executable="rslidar_sdk_node",
                parameters=[{"config_path": str(_AIRY_CONFIG)}],
                output="screen",
            )
        )
    elif lidar != "none":
        raise ValueError(f"lidar must be one of livox/airy/none, got {lidar!r}")

    cmd = [
        python,
        "-m",
        "scripts.tools.realsense_live",
        "--config",
        config,
        "--frames",
        frames,
        "--lidar-msg-type",
        lidar_msg_type,
        "--lidar-roll",
        LaunchConfiguration("lidar_roll").perform(context),
        "--lidar-pitch",
        LaunchConfiguration("lidar_pitch").perform(context),
        "--lidar-yaw",
        LaunchConfiguration("lidar_yaw").perform(context),
        "--z-min",
        LaunchConfiguration("z_min").perform(context),
        "--z-max",
        LaunchConfiguration("z_max").perform(context),
        "--color-bands",
        LaunchConfiguration("color_bands").perform(context),
        "--voxel-size",
        LaunchConfiguration("voxel_size").perform(context),
        "--voxel-alpha",
        LaunchConfiguration("voxel_alpha").perform(context),
    ]
    if lidar != "none":
        cmd += ["--lidar-topic", lidar_topic]
        cmd += ["--lidar-offset", *LaunchConfiguration("lidar_offset").perform(context).split()]
    if view:
        cmd.append("--view")
    if voxel_view:
        cmd.append("--voxel-view")
    if foxglove:
        cmd.append("--foxglove")
    save_map = LaunchConfiguration("save_map").perform(context)
    if save_map:
        cmd += ["--save", save_map]
    voxel_save = LaunchConfiguration("voxel_save").perform(context)
    if voxel_save:
        cmd += ["--voxel-save", voxel_save]
    view_save = LaunchConfiguration("view_save").perform(context)
    if view_save:
        cmd += ["--view-save", view_save]

    mapper = ExecuteProcess(
        cmd=cmd,
        cwd=str(_REPO_ROOT),
        output="screen",
    )
    actions.append(TimerAction(period=delay, actions=[mapper]))
    actions.append(
        RegisterEventHandler(
            OnProcessExit(target_action=mapper, on_exit=[EmitEvent(event=Shutdown())])
        )
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("lidar", default_value="livox", choices=["livox", "airy", "none"]),
            DeclareLaunchArgument("livox_launch", default_value="msg_MID360_launch.py"),
            DeclareLaunchArgument("lidar_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument(
                "lidar_msg_type", default_value="custom", choices=["pointcloud2", "custom"]
            ),
            DeclareLaunchArgument("config", default_value=str(_DEFAULT_CONFIG)),
            DeclareLaunchArgument("python", default_value=str(_VENV_PYTHON)),
            DeclareLaunchArgument("frames", default_value="0"),
            DeclareLaunchArgument("view", default_value="true"),
            DeclareLaunchArgument("voxel_view", default_value="true"),
            DeclareLaunchArgument("foxglove", default_value="false"),
            DeclareLaunchArgument("start_delay", default_value="3.0"),
            DeclareLaunchArgument("lidar_roll", default_value="0.0"),
            DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
            DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
            DeclareLaunchArgument("lidar_offset", default_value="0.0 0.0 0.0"),
            DeclareLaunchArgument("z_min", default_value="-0.1"),
            DeclareLaunchArgument("z_max", default_value="1.0"),
            DeclareLaunchArgument("color_bands", default_value="12"),
            DeclareLaunchArgument("voxel_size", default_value="0.05"),
            DeclareLaunchArgument("voxel_alpha", default_value="0.55"),
            DeclareLaunchArgument("save_map", default_value=""),
            DeclareLaunchArgument("voxel_save", default_value=""),
            DeclareLaunchArgument("view_save", default_value=""),
            OpaqueFunction(function=_launch_setup),
        ]
    )
