import time
import genpy
from typing import Any, override

from influxdb_client_3 import Point

from bridge.alert import AlertSystem
from bridge.database_client import InfluxDB
from bridge.plugin_loader import Plugin
from ros_msgs.custom_msgs.msg._SignalState import SignalState
from std_msgs.msg._Header import Header


class SignalStatusPlugin(Plugin[SignalState]):
    topic_name: str = "/robot/signals/status_update"
    ros_msg_type: type[SignalState] = SignalState
    is_enabled: bool = False
    interval: float = 0.25

    @override
    def callback(self, msg: SignalState):
        """
        Custom handler for SignalState.
        Maps the uint8 'state' to a string tag/field for InfluxDB.
        """
        measurement_name = self.topic_name.removeprefix("/").replace("/", "_")
        SIGNAL_STATE_MAP = {
            SignalState.IDLE: "IDLE",
            SignalState.GREETING: "GREETING",
            SignalState.SPEAKING: "SPEAKING",
            SignalState.MOVING: "MOVING",
            SignalState.STOPPED: "STOPPED",
            SignalState.REVERSE: "REVERSE",
            SignalState.ERROR_MINOR: "ERROR_MINOR",
            SignalState.ERROR_CRITICAL: "ERROR_CRITICAL",
            SignalState.LOW_BATTERY: "LOW_BATTERY",
        }
        measurement_name = "robot_signal_status"

        if msg.state == SignalState.ERROR_CRITICAL:
            AlertSystem.send_slack_message("C09PRS9P08K", "CRITICAL ERROR")

        try:
            state_str = SIGNAL_STATE_MAP.get(msg.state, "UNKNOWN")

            point = Point(measurement_name)

            if self.tags:
                for k, v in self.tags.items():
                    _ = point.tag(k, v) # pyright: ignore [reportUnknownMemberType]

            _ = point.tag("state_label", state_str) # pyright: ignore [reportUnknownMemberType]

            _ = point.field("state_code", int(msg.state)) # pyright: ignore [reportUnknownMemberType]

            client = InfluxDB.get_instance()
            client.write(point) # pyright: ignore [reportUnknownMemberType]
            self.log(f"Send: {measurement_name} -> {state_str} ({msg.state})")

        except Exception as e:
            self.log(f"Failed to write {measurement_name}: {e}")

    @override
    def mock_generator(
        self, state: dict[str, Any]
    ) -> tuple[SignalState, dict[str, Any]]:
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
