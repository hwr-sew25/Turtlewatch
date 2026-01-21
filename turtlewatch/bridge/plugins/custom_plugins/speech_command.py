import time
import genpy
from typing import Any, override

from bridge.plugin_loader import Plugin
from ros_msgs.custom_msgs.speech_in.msg._SpeechCommand import SpeechCommand
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
        msg.command = "mock_command"
        msg.target = f"target_{state['counter']}"
        msg.floor = state["counter"] % 5

        return (msg, state)
