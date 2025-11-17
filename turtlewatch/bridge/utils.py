from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import rospy
import time
from typing import Any


def flatten_ros_message(msg: rospy.Message, prefix="") -> dict[str, Any]:
    items: dict[str, Any] = {}

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
    msg: rospy.Message, measurement_name: str, tags: dict[str,str]
) -> dict[str, Any]:
    flat_msg = flatten_ros_message(msg)
    return {
        "measurement": measurement_name,
        "tags": tags,
        "fields": flat_msg,
        "time": int(time.time()),
    }


# Create a simple test message
msg = Odometry()

# Header
msg.header.frame_id = "odom"
msg.child_frame_id = "base_link"

# Pose
msg.pose.pose.position.x = 1.0
msg.pose.pose.position.y = 2.0
msg.pose.pose.position.z = 0.0
msg.pose.pose.orientation.w = 1.0  # Identity quaternion

# Twist (velocity)
msg.twist.twist.linear.x = 0.5  # Moving forward at 0.5 m/s
msg.twist.twist.angular.z = 0.1  # Rotating at 0.1 rad/s
pose = Pose()

# Set position
pose.position.x = 1.0
pose.position.y = 2.0
pose.position.z = 0.5

# Set orientation (identity quaternion = no rotation)
pose.orientation.x = 0.0
pose.orientation.y = 0.0
pose.orientation.z = 0.0
pose.orientation.w = 1.0

print(flatten_ros_message(pose))
print(ros_msg_to_influx_point(pose, "Pose", ["pose-tag"]))
