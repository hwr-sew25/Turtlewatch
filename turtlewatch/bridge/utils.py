from typing import Any, Dict

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import rospy
import time


def flatten_ros_message(msg: rospy.Message, prefix: str = "") -> Dict[str, Any]:
    items: Dict[str, Any] = {}

    for slot in msg.__slots__:
        key: str = f"{prefix}_{slot}" if prefix else slot
        value: Any = getattr(msg, slot)

        if hasattr(value, "__slots__"):
            items.update(flatten_ros_message(value, key))

        elif isinstance(value, (list, tuple)):
            for i, item in enumerate(value):
                indexed_key = f"{key}[{i}]"
                if hasattr(item, "__slots__"):
                    items.update(flatten_ros_message(item, indexed_key))
                else:
                    items[indexed_key] = item
        else:
            items[key] = value

    return items


def ros_msg_to_influx_point(
    msg: rospy.Message, measurement_name: str, tags: Dict[str, str]
) -> Dict[str, Any]:
    flat_msg = flatten_ros_message(msg)
    return {
        "measurement": measurement_name,
        "tags": tags,
        "fields": flat_msg,
        "time": time.time_ns(),  # Nanosekunden für InfluxDB
    }
