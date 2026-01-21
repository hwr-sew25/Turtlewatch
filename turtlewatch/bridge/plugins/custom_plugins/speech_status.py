import time
import genpy
from typing import Any, override

from bridge.plugin_loader import Plugin
from ros_msgs.custom_msgs.speech_in.msg._SpeechStatus import SpeechStatus
from std_msgs.msg._Header import Header


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

        return (msg, state)
