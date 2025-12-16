import logging
import random
import threading
import time
from typing import Any, Callable

import genpy
from ros_msgs.geometry_msgs.msg import Twist
from ros_msgs.nav_msgs.msg import Odometry
from ros_msgs.std_msgs.msg import Header
from ros_msgs.sensor_msgs.msg import BatteryState
from ros_msgs.actionlib_msgs.msg import GoalStatusArray, GoalStatus, GoalID
from ros_msgs.custom_msgs.msg import SignalState

from bridge.types import Seconds

logger = logging.getLogger("BridgeLogger")


def mock_sub[MsgType: genpy.Message](
    topic_name: str,
    msg_class: type[genpy.Message],
    callback: Callable[[MsgType], None],
    interval: Seconds,
) -> None:
    # NOTE it would be more efficient if these could be asyncio coroutines
    # instead of threads but I don'want to change everything to async since
    # that stuff if managed by ROS if we are not mocking
    # TODO this should probably be refactored to use threading.Event() instead of daemon threads
    thread = threading.Thread(
        target=dispatcher, args=(topic_name, msg_class, callback, interval), daemon=True
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
        "/battery_state": battery_handler,
        "/robot/signals/status_update": signal_status_handler,
        "/move_base/status": move_status_handler,
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
            callback(msg)

            iteration += 1
            next_time = start_time + (iteration * interval)
            sleep_duration = next_time - time.time()
            if sleep_duration > 0:
                time.sleep(sleep_duration)


def cmd_vel_handler(state: dict[str, Any]) -> tuple[Twist, dict[str, Any]]:
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


def signal_status_handler(state: dict[str, Any]) -> tuple[SignalState, dict[str, Any]]:
    """
    Cycles through a logical robot lifecycle:
    Idle -> Greeting -> Speaking -> Moving -> Stopped -> Reverse -> Error -> Low Battery
    """

    # 1. Initialize State Logic
    if not state:
        state = {
            "step_index": 0,
            "counter": 0,
            "duration": 5,  # How many ticks to stay in each state
        }

    # Define the sequence of states to cycle through
    # (Using the constants defined in the .msg file)
    lifecycle = [
        SignalState.IDLE,
        SignalState.GREETING,
        SignalState.SPEAKING,
        SignalState.MOVING,
        SignalState.STOPPED,
        SignalState.REVERSE,
        SignalState.ERROR_MINOR,
        SignalState.LOW_BATTERY,
        SignalState.ERROR_CRITICAL,
    ]

    # 2. Logic to advance state over time
    current_enum = lifecycle[state["step_index"]]
    state["counter"] += 1

    # If we have stayed in this state long enough, move to the next
    if state["counter"] >= state["duration"]:
        state["counter"] = 0
        state["step_index"] = (state["step_index"] + 1) % len(lifecycle)

    # 3. Build Message
    msg = SignalState()
    msg.header = Header()
    msg.header.stamp = genpy.Time.from_sec(time.time())

    msg.state = current_enum

    return (msg, state)


def move_status_handler(
    state: dict[str, Any],
) -> tuple[GoalStatus, dict[str, Any]]:
    """
    Simulates a navigation goal:
    - Starts ACTIVE (moving)
    - Stays ACTIVE for a while
    - Switches to SUCCEEDED (reached goal)
    - Waits, then resets to ACTIVE (new goal)
    
    Returns: A single GoalStatus object (not an Array)
    """

    # 1. Initialize State
    if not state:
        state = {"status": GoalStatus.ACTIVE, "counter": 0, "goal_id_count": 1}

    # 2. State Machine Logic
    # Simulate moving for 20 ticks, then succeed for 10 ticks, then new goal
    TIME_TO_MOVE = 20
    TIME_TO_REST = 10

    state["counter"] += 1

    if state["status"] == GoalStatus.ACTIVE:
        if state["counter"] > TIME_TO_MOVE:
            state["status"] = GoalStatus.SUCCEEDED
            state["counter"] = 0  # Reset counter for the rest phase

    elif state["status"] == GoalStatus.SUCCEEDED:
        if state["counter"] > TIME_TO_REST:
            state["status"] = GoalStatus.ACTIVE
            state["counter"] = 0
            state["goal_id_count"] += 1  # Generate a new Goal ID

    # 3. Build Message (Single GoalStatus)
    msg = GoalStatus()
    
    # Generate timestamp for the GoalID (GoalStatus has no top-level header)
    current_time = genpy.Time.from_sec(time.time())

    msg.status = state["status"]

    # Create a unique GoalID
    msg.goal_id = GoalID()
    msg.goal_id.stamp = current_time
    msg.goal_id.id = f"mock_goal_{state['goal_id_count']}"

    if state["status"] == GoalStatus.ACTIVE:
        msg.text = "Moving to target..."
    elif state["status"] == GoalStatus.SUCCEEDED:
        msg.text = "Goal reached."

    # Return the single message object directly
    return (msg, state)
