import time
import genpy
from typing import Any, override

from bridge.plugin_loader import Plugin
from ros_msgs.custom_msgs.speech_in.msg._SpeechCommand import SpeechCommand
from ros_msgs.custom_msgs.speech_in.msg._SpeechStatus import SpeechStatus
from ros_msgs.custom_msgs.speech_out.msg._SpeechOutput import SpeechOutput
from std_msgs.msg._Header import Header


class SpeechCommandPlugin(Plugin[SpeechCommand]):
    topic_name: str = "/speech_command"
    ros_msg_type: type[SpeechCommand] = SpeechCommand
    is_enabled: bool = True
    tags: dict[str, str] = {}
    interval: float = 0.25

    @override
    def mock_generator(
        self, state: dict[str, Any]
    ) -> tuple[SpeechCommand, dict[str, Any]]:
        if not state:
            state = {"counter": 0}

        state["counter"] += 1

        msg = SpeechCommand()
        msg.header = Header()
        msg.header.stamp = genpy.Time.from_sec(time.time())
        msg.target = f"target_{state['counter']}"
        msg.room_id = f"room_{state['counter']}"
        msg.floor = str(state["counter"] % 5)
        msg.wing = "A"

        return (msg, state)


class SpeechStatusPlugin(Plugin[SpeechStatus]):
    topic_name: str = "/speech_output"
    ros_msg_type: type[SpeechStatus] = SpeechStatus
    is_enabled: bool = True
    tags: dict[str, str] = {}
    interval: float = 0.25

    @override
    def mock_generator(
        self, state: dict[str, Any]
    ) -> tuple[SpeechStatus, dict[str, Any]]:
        if not state:
            state = {"counter": 0, "levels": ["info", "warning", "error"]}

        state["counter"] += 1
        level = state["levels"][state["counter"] % len(state["levels"])]

        msg = SpeechStatus()
        msg.header = Header()
        msg.header.stamp = genpy.Time.from_sec(time.time())
        msg.level = level
        msg.message = f"mock_message_{state['counter']}"
        msg.event = "speech_status"
        msg.transcript = f"transcript_{state['counter']}"

        return (msg, state)


class SpeechOutputPlugin(Plugin[SpeechOutput]):
    topic_name: str = "/speech_out/message"
    ros_msg_type: type[SpeechOutput] = SpeechOutput
    is_enabled: bool = True
    tags: dict[str, str] = {}
    interval: float = 0.25

    @override
    def mock_generator(
        self, state: dict[str, Any]
    ) -> tuple[SpeechOutput, dict[str, Any]]:
        if not state:
            state = {"counter": 0}

        state["counter"] += 1

        msg = SpeechOutput()
        msg.header = Header()
        msg.header.stamp = genpy.Time.from_sec(time.time())
        msg.text = f"mock_output_{state['counter']}"

        return (msg, state)
