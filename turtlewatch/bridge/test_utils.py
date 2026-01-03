from ros_msgs.geometry_msgs.msg import Pose
from bridge.utils import flatten_ros_message, ros_msg_to_influx_point
import pytest


def test_flatten_ros_message():
    pose = Pose()

    pose.position.x = 1.0
    pose.position.y = 2.0
    pose.position.z = 0.5

    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0

    flat = flatten_ros_message(pose)
    flat_correct = {
        "position_x": 1.0,
        "position_y": 2.0,
        "position_z": 0.5,
        "orientation_x": 0.0,
        "orientation_y": 0.0,
        "orientation_z": 0.0,
        "orientation_w": 1.0,
    }
    assert flat == flat_correct


def test_ros_msg_to_influx_point(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setattr("time.time_ns", lambda: 1765540015249344480)

    pose = Pose()

    pose.position.x = 1.0
    pose.position.y = 2.0
    pose.position.z = 0.5

    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0

    tags = {"type": "robot"}
    point = ros_msg_to_influx_point(pose, "pose", tags)

    point_correct = {
        "measurement": "pose",
        "tags": {"type": "robot"},
        "fields": {
            "position_x": 1.0,
            "position_y": 2.0,
            "position_z": 0.5,
            "orientation_x": 0.0,
            "orientation_y": 0.0,
            "orientation_z": 0.0,
            "orientation_w": 1.0,
        },
        "time": 1765540015249344480,
    }
    assert point == point_correct
