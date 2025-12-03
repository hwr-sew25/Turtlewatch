import logging
import random
import threading
import time
from typing import Any, Callable

import genpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from bridge.types import Seconds

logger = logging.getLogger("BridgeLogger")


def mock_sub(
    topic_name: str,
    msg_class: type[genpy.Message],
    callback: Callable[[genpy.Message], None],
    interval: Seconds,
) -> None:
    # NOTE it would be more efficient if these could be asyncio coroutines
    # instead of threads but I don'want to change everything to async since
    # that stuff if managed by ROS if we are not mocking
    thread = threading.Thread(
        target=dispatcher, args=(topic_name, msg_class, callback, interval)
    )
    thread.start()


def dispatcher(
    topic_name: str,
    msg_class: type[genpy.Message],
    callback: Callable[[genpy.Message], None],
    interval: Seconds,
) -> None:
    topics = {
        "/cmd_vel": cmd_vel_handler,
        "/odom": odom_handler,
    }

    handler = topics.get(topic_name)
    if not handler:
        logger.error(f"Mock handler for {topic_name} not implemented yet")
    else:
        iteration = 0
        start_time = time.time()
        state: dict[str,Any] = {}
        while True:
            # This takes the function out of the topics dict and calls it
            (msg, state) = topics[topic_name](state, msg_class, callback, interval)
            callback(msg)

            iteration += 1
            next_time = start_time + (iteration * interval)
            sleep_duration = next_time - time.time()
            if sleep_duration > 0:
                time.sleep(sleep_duration)


def cmd_vel_handler(
    state,
    msg_class: type[genpy.Message],
    callback: Callable[[genpy.Message], None],
    interval: Seconds,
) -> tuple[Twist, dict[str, Any]]:
    """Simulates a robot moving forward and turning slightly."""
    msg = Twist()
    msg.linear.x = 0.5 + random.uniform(-0.05, 0.05)
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.1

    return (msg, {})


def odom_handler(
    state: dict[str, Any],
    msg_class: type[genpy.Message],
    callback: Callable[[genpy.Message], None],
    interval: Seconds,
) -> tuple[Odometry, dict[str, Any]]:
    """Simulates odometry data (position and velocity)."""

    if not state:
        state = {"x_pos": 0.0, "y_pos": 0.0}

    msg = Odometry()

    msg.header = Header()
    msg.header.stamp = genpy.Time.from_sec(time.time())
    msg.header.frame_id = "odom"
    msg.child_frame_id = "base_link"

    # 2. Simulate Position (Pose) - moving in a simple line for testing
    state["x_pos"] += 0.05
    msg.pose.pose.position.x = state["x_pos"]
    msg.pose.pose.position.y = state["y_pos"]
    msg.pose.pose.position.z = 0.0

    # Valid Quaternion (identity = no rotation)
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = 0.0
    msg.pose.pose.orientation.w = 1.0

    # 3. Simulate Velocity (Twist)
    msg.twist.twist.linear.x = 0.5
    msg.twist.twist.angular.z = 0.0

    return (msg, state)
