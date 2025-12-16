import logging
import math
import random
import threading
import time
from typing import Any, Callable

import genpy
from ros_msgs.geometry_msgs.msg import Twist
from ros_msgs.nav_msgs.msg import Odometry
from ros_msgs.std_msgs.msg import Header
from ros_msgs.sensor_msgs.msg import BatteryState

from bridge.types import Seconds

logger = logging.getLogger("BridgeLogger")


def mock_sub[MsgType: genpy.Message](
    topic_name: str,
    msg_class: type[genpy.Message],
    callback: Callable[[MsgType, str, dict[str, str] | None], None],
    interval: Seconds,
) -> None:
    # NOTE it would be more efficient if these could be asyncio coroutines
    # instead of threads but I don'want to change everything to async since
    # that stuff if managed by ROS if we are not mocking
    # TODO this should probably be refactored to use threading.Event() instead of daemon threads
    thread = threading.Thread(
        target=dispatcher, args=(topic_name, msg_class, callback, interval),
        daemon=True
    )
    thread.start()


def dispatcher(
    topic_name: str,
    msg_class: type[genpy.Message],
    callback: Callable[[genpy.Message, str, dict[str, str] | None], None],
    interval: Seconds,
) -> None:
    topics = {
        "/cmd_vel": cmd_vel_handler,
        "/odom": odom_handler,
        "/battery_state": battery_handler,
    }

    handler = topics.get(topic_name)
    tags = {}
    if not handler:
        logger.error(f"Mock handler for {topic_name} not implemented yet")
    else:
        iteration = 0
        start_time = time.time()
        state: dict[str, Any] = {}
        while True:
            # This takes the function out of the topics dict and calls it
            (msg, state) = topics[topic_name](state)
            callback(msg, topic_name, tags)

            iteration += 1
            next_time = start_time + (iteration * interval)
            sleep_duration = next_time - time.time()
            if sleep_duration > 0:
                time.sleep(sleep_duration)


def cmd_vel_handler(state: dict[str, Any]) -> tuple[Twist, dict[str, Any]]:
    """Simulates a robot moving forward and turning slightly."""
    msg = Twist()
    msg.linear.x = 0.3 + random.uniform(-0.05, 0.05)
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.2 + random.uniform(-0.05, 0.05)

    return (msg, {})


def odom_handler(
    state: dict[str, Any],
) -> tuple[Odometry, dict[str, Any]]:
    """Simulates odometry data (position and velocity) within map boundaries.

    Map boundaries:
    - x: -7 to 7
    - y: -13 to 13

    Robot drives in a smooth rectangular pattern.
    """
    # Map boundaries with margin
    X_MIN, X_MAX = -5.0, 5.0
    Y_MIN, Y_MAX = -10.0, 10.0

    if not state:
        state = {
            "x_pos": 0.0,
            "y_pos": 0.0,
            "theta": 0.0,  # heading angle in radians
            "target_theta": 0.0,  # target heading for smooth turning
            "speed": 0.1,  # slower speed for smoother movement
        }

    msg = Odometry()

    msg.header = Header()
    msg.header.stamp = genpy.Time.from_sec(time.time())
    msg.header.frame_id = "odom"
    msg.child_frame_id = "base_link"

    speed = state["speed"]
    theta = state["theta"]

    # Smoothly interpolate towards target heading
    theta_diff = state["target_theta"] - theta
    if abs(theta_diff) > 0.01:
        state["theta"] += theta_diff * 0.1  # smooth turning

    # Calculate new position
    new_x = state["x_pos"] + speed * math.cos(state["theta"])
    new_y = state["y_pos"] + speed * math.sin(state["theta"])

    # Check boundaries and set new target heading
    if new_x >= X_MAX:
        state["target_theta"] = math.pi  # turn left (180°)
        new_x = X_MAX
    elif new_x <= X_MIN:
        state["target_theta"] = 0.0  # turn right (0°)
        new_x = X_MIN
    elif new_y >= Y_MAX:
        state["target_theta"] = -math.pi / 2  # turn down (-90°)
        new_y = Y_MAX
    elif new_y <= Y_MIN:
        state["target_theta"] = math.pi / 2  # turn up (90°)
        new_y = Y_MIN

    state["x_pos"] = new_x
    state["y_pos"] = new_y

    msg.pose.pose.position.x = state["x_pos"]
    msg.pose.pose.position.y = state["y_pos"]
    msg.pose.pose.position.z = 0.0

    # Convert heading to quaternion (rotation around z-axis)
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = math.sin(state["theta"] / 2)
    msg.pose.pose.orientation.w = math.cos(state["theta"] / 2)

    # Velocity
    msg.twist.twist.linear.x = speed
    msg.twist.twist.angular.z = 0.0

    return (msg, state)


def battery_handler(state: dict[str, Any]) -> tuple[BatteryState, dict[str, Any]]:
    """
    Simulates a battery draining over time.
    Returns sensor_msgs/BatteryState.
    """

    # 1. Initialize State if it's the first run
    if not state:
        state = {
            "percentage": 1.0,  # 1.0 = 100%
            "voltage": 12.3,  # Typical full charge for a 3S LiPo
        }

    msg = BatteryState()
    msg.header = Header()
    msg.header.stamp = genpy.Time.from_sec(time.time())

    # 2. Simulate Draining
    # Decrease percentage by 0.5% every callback
    state["percentage"] -= 0.005
    # Simulate voltage drop roughly proportional to percentage
    state["voltage"] -= 0.01

    # 3. Reset logic (infinite loop simulation)
    if state["percentage"] <= 0.0:
        state["percentage"] = 1.0
        state["voltage"] = 12.3

    # 4. Populate Message Fields
    msg.voltage = state["voltage"]
    msg.percentage = state["percentage"]
    msg.current = -1.5  # Negative current indicates discharging
    msg.charge = 4.0  # Ah current capacity
    msg.capacity = 4.0  # Ah total capacity
    msg.design_capacity = 4.0

    # Use standard constants from the message definition
    msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
    msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
    msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
    msg.present = True

    return (msg, state)
