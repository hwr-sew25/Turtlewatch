import logging
import time
import genpy
from typing import Any, override

from influxdb_client_3 import Point
from bridge.database_client import InfluxDB
from bridge.plugin_loader import Plugin
from bridge.stats import StatsTracker
from ros_msgs.actionlib_msgs.msg._GoalID import GoalID
from ros_msgs.actionlib_msgs.msg._GoalStatus import GoalStatus


class MoveStatusPlugin(Plugin[GoalStatus]):
    topic_name: str = "/move_base/status"
    ros_msg_type: type[GoalStatus] = GoalStatus
    is_enabled: bool = True
    tags: dict[str, str] = {}
    interval: float = 0.25

    @override
    def callback(self, msg: GoalStatus) -> None:
        """
        Custom handler for Move/GoalStatus.
        Maps the uint8 'status' to a string tag/field for InfluxDB.
        """
        measurement_name = self.topic_name.removeprefix("/").replace("/", "_")
        GOAL_STATUS_MAP = {
            GoalStatus.PENDING: "PENDING",
            GoalStatus.ACTIVE: "ACTIVE",
            GoalStatus.PREEMPTED: "PREEMPTED",
            GoalStatus.SUCCEEDED: "SUCCEEDED",
            GoalStatus.ABORTED: "ABORTED",
            GoalStatus.REJECTED: "REJECTED",
            GoalStatus.PREEMPTING: "PREEMPTING",
            GoalStatus.RECALLING: "RECALLING",
            GoalStatus.RECALLED: "RECALLED",
            GoalStatus.LOST: "LOST",
        }

        session = StatsTracker.get_session()
        session.completion_status = msg.status
        self.log(f"Received goal status: {msg.status}")

        if msg.status in [
            GoalStatus.ABORTED,
            GoalStatus.SUCCEEDED,
            GoalStatus.RECALLED,
            GoalStatus.LOST,
        ]:
            StatsTracker.stop_current_session()

        try:
            # Get the string representation, default to UNKNOWN if status ID is invalid
            state_str = GOAL_STATUS_MAP.get(msg.status, "UNKNOWN")

            point = Point(measurement_name)

            # Apply existing tags
            if self.tags:
                for k, v in self.tags.items():
                    point.tag(k, v)

            # Add specific status data
            point.tag("state_label", state_str)
            point.field("state_code", int(msg.status))

            client = InfluxDB.get_instance()
            client.write(point)
            self.log(f"Send: {measurement_name} -> {state_str} ({msg.status})")

        except Exception as e:
            self.log(f"Failed to write {measurement_name}: {e}", logging.ERROR)

    @override
    def mock_generator(
        self, state: dict[str, Any]
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
