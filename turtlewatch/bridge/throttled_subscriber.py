import os
import time
from typing import Callable, TypeAlias
import genpy
import rospy

from bridge.mock import mock_sub
from bridge.types import Seconds


class ThrottledSubscriber[MsgType: genpy.Message]:
    def __init__(
        self,
        topic_name: str,
        msg_class: type[MsgType],
        callback: Callable[[MsgType], None],
        interval: Seconds,
    ):
        self.topic_name: str = topic_name
        self.msg_class: type[MsgType] = msg_class
        self.callback: Callable[[MsgType], None] = callback
        self.last_time: float = time.time()
        self.interval: Seconds = interval

        mock = os.getenv("MOCK")
        if mock and mock.lower() == "true":
            # NOTE we pass the real callback so we don't throttle twice 
            mock_sub(topic_name, msg_class, callback, interval) 
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
        if self.should_run():
            self.callback(msg)
