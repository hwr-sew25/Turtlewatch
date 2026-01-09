import time
import genpy
from typing import Any, override

from influxdb_client_3 import Point

from bridge.alert import AlertSystem
from bridge.database_client import InfluxDB
from bridge.plugin_loader import Plugin
from ros_msgs.custom_msgs.msg._SignalState import SignalState
from ros_msgs.custom_msgs.msg._SignalStatusUpdate import SignalStatusUpdate
from std_msgs.msg._Header import Header


class SignalStatusPlugin(Plugin[SignalStatusUpdate]):
    """
    Handles /signals/status_update messages that carry current + previous state.
    """

    topic_name: str = "/signals/status_update"
    ros_msg_type: type[SignalStatusUpdate] = SignalStatusUpdate
    is_enabled: bool = True
    interval: float = 0.25

    # Mapping of SignalState IDs to human-readable labels (fallback if msg has no name)
    STATE_LABELS: dict[int, str] = {
        SignalState.GREETING: "GREETING",
        SignalState.IDLE: "IDLE",
        SignalState.BUSY: "BUSY",
        SignalState.STOP_BUSY: "STOP_BUSY",
        SignalState.ERROR_MINOR_STUCK: "ERROR_MINOR_STUCK",
        SignalState.ERROR_MINOR_NAV: "ERROR_MINOR_NAV",
        SignalState.ROOM_NOT_FOUND: "ROOM_NOT_FOUND",
        SignalState.ERROR_MAJOR: "ERROR_MAJOR",
        SignalState.LOW_BATTERY: "LOW_BATTERY",
        SignalState.MOVE: "MOVE",
        SignalState.START_MOVE: "START_MOVE",
        SignalState.STOP_MOVE: "STOP_MOVE",
        SignalState.GOAL_REACHED: "GOAL_REACHED",
        SignalState.REVERSE: "REVERSE",
        SignalState.SPEAKING: "SPEAKING",
    }

    @override
    def callback(self, msg: SignalStatusUpdate):
        measurement_name = "robot_signal_status"

        # Use names if provided, otherwise fall back to mapping
        current_label = msg.current_state_name or self.STATE_LABELS.get(
            msg.current_state, "UNKNOWN"
        )
        previous_label = msg.previous_state_name or self.STATE_LABELS.get(
            msg.previous_state, "UNKNOWN"
        )

        if msg.current_state in (
            SignalState.ERROR_MAJOR,
            SignalState.ERROR_MINOR_STUCK,
            SignalState.ERROR_MINOR_NAV,
        ):
            AlertSystem.send_slack_message(
                "C09PRS9P08K", f"Signal error: {current_label} ({msg.current_state})"
            )

        try:
            point = Point(measurement_name)

            for k, v in self.tags.items():
                point.tag(k, v)

            point.tag("current_label", current_label)
            point.tag("previous_label", previous_label)
            point.field("current_state", int(msg.current_state))
            point.field("previous_state", int(msg.previous_state))
            point.field("info", msg.info or "")

            client = InfluxDB.get_instance()
            client.write(point)  # pyright: ignore[reportUnknownMemberType]
            self.log(
                f"Send: {measurement_name} -> {current_label} ({msg.current_state}) "
                f"prev={previous_label} ({msg.previous_state})"
            )

        except Exception as e:
            self.log(f"Failed to write {measurement_name}: {e}")

    @override
    def mock_generator(
        self, state: dict[str, Any]
    ) -> tuple[SignalStatusUpdate, dict[str, Any]]:
        """
        Generates a realistic status update with current/previous states and info.
        """

        if not state:
            state = {
                "step_index": 0,
                "counter": 0,
                "duration": 5,
            }

        lifecycle = [
            SignalState.IDLE,
            SignalState.GREETING,
            SignalState.SPEAKING,
            SignalState.MOVE,
            SignalState.STOP_MOVE,
            SignalState.GOAL_REACHED,
            SignalState.REVERSE,
            SignalState.ERROR_MINOR_STUCK,
            SignalState.LOW_BATTERY,
        ]

        previous_enum = lifecycle[state["step_index"]]
        state["counter"] += 1

        if state["counter"] >= state["duration"]:
            state["counter"] = 0
            state["step_index"] = (state["step_index"] + 1) % len(lifecycle)

        current_enum = lifecycle[state["step_index"]]

        msg = SignalStatusUpdate()
        msg.header = Header()
        msg.header.stamp = genpy.Time.from_sec(time.time())
        msg.state_changed_at = genpy.Time.from_sec(time.time())
        msg.current_state = current_enum
        msg.previous_state = previous_enum
        msg.current_state_name = self.STATE_LABELS.get(current_enum, "UNKNOWN")
        msg.previous_state_name = self.STATE_LABELS.get(previous_enum, "UNKNOWN")
        msg.info = "Battery low" if current_enum == SignalState.LOW_BATTERY else "OK"

        return (msg, state)
