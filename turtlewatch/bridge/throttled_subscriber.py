from typing import Callable, Generic, Type, TypeVar

import genpy
import rospy

MsgType = TypeVar('MsgType', bound=genpy.Message)


class ThrottledSubscriber(Generic[MsgType]):
    def __init__(
        self,
        topic_name: str,
        msg_class: Type[MsgType],
        callback: Callable[[MsgType], None],
        interval: rospy.Duration,
    ):
        self.topic_name: str = topic_name
        self.msg_class: Type[MsgType] = msg_class
        self.callback: Callable[[MsgType], None] = callback
        self.last_time: rospy.Time = rospy.Time.now()
        self.interval: rospy.Duration = interval

        _ = rospy.Subscriber(topic_name, msg_class, self._internal_callback)

    def should_run(self) -> bool:
        """Returns True if the action should run again"""
        now = rospy.Time.now()
        if now - self.last_time >= self.interval:
            self.last_time = now
            return True
        return False

    def _internal_callback(self, msg: MsgType) -> None:
        if self.should_run():
            self.callback(msg)
