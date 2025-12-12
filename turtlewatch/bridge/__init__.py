"""
Bridge package initialization.
Automatically configures Python path to use local genpy, genmsg, and ros_messages.
This ensures the bridge works in CI without requiring system ROS packages.
"""

import sys
from pathlib import Path

# Get the libs directory relative to this file
_bridge_dir = Path(__file__).parent
_libs_dir = _bridge_dir.parent / "libs"

# Add local ros_messages to Python path (MUST be first to override system packages)
_ros_messages_path = str(_libs_dir / "ros_messages")
if _ros_messages_path in sys.path:
    sys.path.remove(_ros_messages_path)
sys.path.insert(0, _ros_messages_path)

# Add local genmsg to Python path (override system version)
_genmsg_path = str(_libs_dir / "genmsg" / "src")
if _genmsg_path in sys.path:
    sys.path.remove(_genmsg_path)
sys.path.insert(0, _genmsg_path)

# Add local genpy to Python path (override system version)
_genpy_path = str(_libs_dir / "genpy" / "src")
if _genpy_path in sys.path:
    sys.path.remove(_genpy_path)
sys.path.insert(0, _genpy_path)
