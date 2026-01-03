import time
from typing import Any, override
from bridge.plugin_loader import Plugin
from bridge.types import ExampleMessageType


class ExamplePlugin(Plugin[ExampleMessageType]):
    topic_name: str = "/example"
    ros_msg_type: type[ExampleMessageType] = ExampleMessageType
    is_enabled: bool = False
    tags: dict[str, str] = {}
    interval: float = 0.25

    @override
    def mock_generator(
        self, state: dict[str, Any]
    ) -> tuple[ExampleMessageType, dict[str, Any]]:
        # Initialize state if this is the first run
        if not state:
            state = {"start_time": time.time()}

        msg = ExampleMessageType()

        return (msg, state)
