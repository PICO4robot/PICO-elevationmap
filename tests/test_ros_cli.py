"""pico-elevationmap-ros import safety: help and errors without ROS installed."""

from __future__ import annotations

import os
import subprocess
import sys

_SCRIPT = r"""
import sys

_BLOCKED = {
    "rclpy",
    "geometry_msgs",
    "nav_msgs",
    "sensor_msgs",
    "std_srvs",
    "grid_map_msgs",
    "builtin_interfaces",
}


class _BlockRos:
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split(".")[0] in _BLOCKED:
            raise ModuleNotFoundError(f"No module named {fullname!r}", name=fullname)
        return None


sys.meta_path.insert(0, _BlockRos())

import pico_elevationmap_ros.node as node

parser = node._build_parser()
assert str(parser.parse_args(["--config", "/tmp/example.yaml"]).config) == "/tmp/example.yaml"
assert str(parser.parse_args(["--config=/tmp/example.yaml"]).config) == "/tmp/example.yaml"

try:
    node.main(["--help"])
except SystemExit as exc:
    assert exc.code == 0
else:
    raise AssertionError("--help should exit with status 0")

try:
    node.main(["--config", "/tmp/example.yaml"])
except SystemExit as exc:
    message = str(exc.code)
    assert "ROS 2" in message
    assert "source" in message
else:
    raise AssertionError("main should fail without ROS")
"""


def test_import_help_and_error_without_ros(tmp_path) -> None:
    env = {key: value for key, value in os.environ.items() if key != "PYTHONPATH"}
    proc = subprocess.run(
        [sys.executable, "-c", _SCRIPT],
        cwd=tmp_path,
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )
    assert proc.returncode == 0, proc.stderr
