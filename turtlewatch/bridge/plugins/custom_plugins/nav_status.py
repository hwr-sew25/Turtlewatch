import time
import genpy
from typing import Any, override

from bridge.plugin_loader import Plugin
from ros_msgs.custom_msgs.movement.msg._NavStatus import NavStatus
from std_msgs.msg._Header import Header


class NavStatusPlugin(Plugin[NavStatus]):
    topic_name: str = "/navbot/nav_status"
    ros_msg_type: type[NavStatus] = NavStatus
    is_enabled: bool = True
    tags: dict[str, str] = {}
    interval: float = 0.25

    @override
    def mock_generator(self, state: dict[str, Any]) -> tuple[NavStatus, dict[str, Any]]:
        if not state:
            state = {
                "index": 0,
                "states": [
                    NavStatus.READY,
                    NavStatus.MOVING_TO_TARGET,
                    NavStatus.ARRIVED,
                    NavStatus.WAITING_FOR_SPEECH,
                    NavStatus.RETURNING_TO_START,
                    NavStatus.AT_START,
                    NavStatus.EMERGENCY_STOP,
                    NavStatus.FAILED,
                ],
            }

        state["index"] = (state["index"] + 1) % len(state["states"])

        msg = NavStatus()
        msg.header = Header()
        msg.header.stamp = genpy.Time.from_sec(time.time())
        msg.state = state["states"][state["index"]]
        msg.target_id = "mock_target"
        msg.detail = "mock_detail"

        return (msg, state)
