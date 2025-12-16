"""Configure sys.path so bundled ROS Python packages can be imported without a system ROS install."""

import sys
from pathlib import Path


def _prepend(path: Path) -> None:
    """Insert the path at the front of sys.path, removing any previous occurrence."""
    path_str = str(path)
    if path_str in sys.path:
        sys.path.remove(path_str)
    sys.path.insert(0, path_str)


_bridge_dir = Path(__file__).resolve().parent
_project_root = _bridge_dir.parent
_libs_dir = _project_root / "libs"

# Catkin helpers bundled in libs/.
_prepend(_libs_dir / "catkin" / "python")

# Core generation tools bundled in libs/.
_prepend(_libs_dir / "genmsg" / "src")
_prepend(_libs_dir / "genpy" / "src")


# ROS runtime Python packages bundled under libs/ros_comm.
_ros_comm_paths = [
    _libs_dir / "ros_comm" / "clients" / "rospy" / "src",
    _libs_dir / "ros_comm" / "clients" / "roscpp" / "src",
    _libs_dir / "ros_comm" / "clients" / "roscpp" / "srv",
    _libs_dir / "ros_comm" / "tools" / "rosgraph" / "src",
    _libs_dir / "ros_comm" / "tools" / "rosmsg" / "src",
    _libs_dir / "ros_comm" / "tools" / "rosparam" / "src",
    _libs_dir / "ros_comm" / "tools" / "rosnode" / "src",
    _libs_dir / "ros_comm" / "tools" / "rostopic" / "src",
    _libs_dir / "ros_comm" / "tools" / "roslaunch" / "src",
    _libs_dir / "ros_comm" / "tools" / "rosservice" / "src",
    _libs_dir / "ros_comm" / "tools" / "rosbag" / "src",
    _libs_dir / "ros_comm" / "tools" / "rosbag_storage" / "src",
    _libs_dir / "ros_comm" / "tools" / "rosmaster" / "src",
    _libs_dir / "ros_comm" / "tools" / "topic_tools" / "src",
    _libs_dir / "ros_comm" / "utilities" / "message_filters" / "src",
    _libs_dir / "ros_comm" / "utilities" / "roslz4" / "src",
    _libs_dir / "ros_comm" / "utilities" / "roswtf" / "src",
]

for path in _ros_comm_paths:
    if path.exists():
        _prepend(path)

# Core ROS Python support (roslib, roslang, etc.)
_ros_core_paths = [
    _libs_dir / "ros" / "core" / "roslib" / "src",
    _libs_dir / "ros" / "core" / "roslang" / "src",
]

for path in _ros_core_paths:
    if path.exists():
        _prepend(path)

# Generated ROS messages live in the repository root. Place this last so it wins
# when a system/global ROS installation is present.
_prepend(_project_root / "ros_msgs")
