import time
import genpy
from typing import Any, override

from bridge.plugin_loader import Plugin
from ros_msgs.custom_msgs.movement.msg._EmergencyStop import EmergencyStop
from std_msgs.msg._Header import Header


class EmergencyStopPlugin(Plugin[EmergencyStop]):
    topic_name: str = "/movement/emergency_top"
    ros_msg_type: type[EmergencyStop] = EmergencyStop
    is_enabled: bool = True
    tags: dict[str, str] = {}
    interval: float = 0.25

    @override
    def mock_generator(
        self, state: dict[str, Any]
    ) -> tuple[EmergencyStop, dict[str, Any]]:
        if not state:
            state = {"counter": 0, "toggle_after": 8}

        state["counter"] += 1
        is_emergency_stop = (state["counter"] // state["toggle_after"]) % 2 == 1

        msg = EmergencyStop()
        msg.header = Header()
        msg.header.stamp = genpy.Time.from_sec(time.time())
        msg.is_emergency_stop = is_emergency_stop

        return (msg, state)
