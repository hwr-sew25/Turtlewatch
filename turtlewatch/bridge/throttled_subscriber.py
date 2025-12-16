import os
import time
from typing import Callable
import genpy
import rospy

from bridge.mock import mock_sub
from bridge.stats import StatsTracker
from bridge.types import Seconds


class ThrottledSubscriber[MsgType: genpy.Message]:
    def __init__(
        self,
        topic_name: str,
        msg_class: type[MsgType],
        callback: Callable[[MsgType, str, dict[str, str] | None], None],
        interval: Seconds,
        tags: dict[str, str] | None = None,
    ):
        self.topic_name: str = topic_name
        self.msg_class: type[MsgType] = msg_class
        self.callback: Callable[[MsgType, str, dict[str, str] | None], None] = callback
        self.last_time: float = time.time()
        self.interval: Seconds = interval
        self.tags: dict[str, str] | None = tags

        mock = os.getenv("MOCK")
        if mock and mock.lower() == "true":
            mock_sub(topic_name, msg_class, self._internal_callback, interval)
        else:
            _ = rospy.Subscriber(topic_name, msg_class, self._internal_callback)

    def should_run(self) -> bool:
        """Returns True if the action should run again based on system time."""
        now = time.time()
        if now - self.last_time >= self.interval:
            self.last_time = now
            return True
        return False

    def _internal_callback(self, msg: MsgType) -> None:
        StatsTracker.update_number_of_messages(1)
        # NOTE don't throttle twice
        mock = os.getenv("MOCK")
        if mock and mock.lower() == "true":
            self.callback(msg, self.topic_name, self.tags)
        else:
            if self.should_run():
                self.callback(msg, self.topic_name, self.tags)
