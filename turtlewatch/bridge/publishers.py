""" Publisher module

Usage:
    from bridge.publishers import register_publishers, get_publisher, Publisher
    from sensor_msgs.msg import BatteryState

    # 1. Initialization (Run this once at startup, e.g., in main.py)
    register_publishers()

    # 2. Sending a message (Run this anywhere)
    pub = get_publisher(Publisher.BATTERY_STATE)
    if pub:
        msg = BatteryState()
        msg.voltage = 12.5
        pub.publish(msg)
"""
from __future__ import annotations
from enum import Enum
import logging
import rospy
from sensor_msgs.msg import BatteryState

logger = logging.getLogger("BridgeLogger")

_publishers: dict[Publisher, rospy.Publisher] = {}

class Publisher(Enum):
    BATTERY_STATE = "monitoring/battery_state"

def register_publishers() -> None:
    """
    Create all publishers here, since we have no init function or smth in our plugins
    """
    # NOTE would be nicer to have it in the plugins but since we only need 1-2 publishers,
    # i don't care
    _publishers[Publisher.BATTERY_STATE] = rospy.Publisher(Publisher.BATTERY_STATE.value, BatteryState, queue_size=10)
    logger.warning(f"Registered publisher: Key: battery_state -> Topic: battery_state")

def get_publisher(key: Publisher) -> rospy.Publisher | None:
    """
    Retrieves a publisher by its key. 
    Returns None if the publisher does not exist
    """
    if key not in _publishers:
        logger.warning(f"Publisher {key} does not exist.")
        return None
    
    return _publishers[key]
