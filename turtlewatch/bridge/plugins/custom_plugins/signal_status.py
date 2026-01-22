import time
import genpy
from typing import Any, override

from bridge.alert import AlertSystem
from bridge.plugin_loader import Plugin
from std_msgs.msg import Header
from ros_msgs.custom_msgs.signal.msg._SignalState import SignalState
from ros_msgs.custom_msgs.signal.msg._SignalStatusUpdate import SignalStatusUpdate


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
    def callback(self, msg: SignalStatusUpdate) -> None:
        if msg.current_state in (
            SignalState.ERROR_MAJOR,
            SignalState.ERROR_MINOR_STUCK,
            SignalState.ERROR_MINOR_NAV,
        ):
            AlertSystem.send_slack_message(
                "C09PRS9P08K",
                f"Signal error: {msg.current_state_name}, Error: {msg.info}",
            )

        super().callback(msg)

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
