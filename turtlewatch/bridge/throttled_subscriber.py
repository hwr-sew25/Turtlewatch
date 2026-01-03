import os
import time
import genpy
import rospy

from bridge.mock import mock_subscriber
from bridge.plugin_loader import Plugin
from bridge.stats import StatsTracker


class ThrottledSubscriber[MsgType: genpy.Message]:
    def __init__(
        self,
        plugin: Plugin[genpy.Message],
    ):
        self.plugin: Plugin[genpy.Message] = plugin
        self.last_time: float = time.time()

        mock = os.getenv("MOCK")
        if mock and mock.lower() == "true":
            mock_subscriber(plugin)
        else:
            _ = rospy.Subscriber(
                plugin.topic_name, plugin.ros_msg_type, self._internal_callback
            )

    def should_run(self) -> bool:
        """Returns True if the action should run again based on system time."""
        now = time.time()
        if now - self.last_time >= self.plugin.interval:
            self.last_time = now
            return True
        return False

    def _internal_callback(self, msg: MsgType) -> None:
        session = StatsTracker.get_session()
        session.number_of_messages += 1
        # NOTE don't throttle twice
        mock = os.getenv("MOCK")
        if mock and mock.lower() == "true":
            self.plugin.callback(msg)
        else:
            if self.should_run():
                self.plugin.callback(msg)
