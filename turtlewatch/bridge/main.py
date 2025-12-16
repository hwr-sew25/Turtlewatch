import os
import sys
import threading
from dotenv import load_dotenv
from actionlib_msgs.msg import GoalStatusArray
import genpy
from influxdb_client_3 import Point
import rospy
from ros_msgs.geometry_msgs.msg import Twist
import logging
from bridge.database_client import DatabaseClient
from bridge.throttled_subscriber import ThrottledSubscriber
from bridge.utils import ros_msg_to_influx_point
from bridge.types import Seconds
from ros_msgs.nav_msgs.msg._Odometry import Odometry
from ros_msgs.sensor_msgs.msg._BatteryState import BatteryState
from ros_msgs.custom_msgs.msg._SignalState import SignalState

logger = logging.getLogger("BridgeLogger")


def setup_logger():
    logger.setLevel(logging.INFO)

    handler = logging.StreamHandler(sys.stdout)
    formatter = logging.Formatter(
        "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    )
    handler.setFormatter(formatter)
    logger.addHandler(handler)

    # I guess this keeps it logging to /rosout
    logger.propagate = True


def main():
    logger.info("Connecting to InfluxDB...")

    with open("../influxdb_token.txt", "r") as file:
        db_token = file.read().strip()

    DatabaseClient.intialize(
        host="http://localhost:8181", database="dev", token=db_token
    )
    logger.info("Successfully connected to InfluxDB (database: dev)")

    topics_config = {
        # ros_topic_name: (MessageType, Callback function)
        "/cmd_vel": (Twist, generic_callback),
        "/odom": (Odometry, generic_callback),
        "/battery_state": (BatteryState, generic_callback),
        "/move_base/status": (GoalStatusArray, generic_callback),
        "/robot/signals/status_update": (SignalState, signal_state_callback),
    }

    for topic_name, (msg_class, callback_handler) in topics_config.items():
        _ = ThrottledSubscriber(
            topic_name=topic_name,
            msg_class=msg_class,
            callback=callback_handler,
            interval=Seconds(0.25),
        )


def generic_callback(msg: genpy.Message, topic_name: str, tags: dict[str, str] | None):
    measurement_name = topic_name.removeprefix("/").replace("/", "_")
    try:
        point = ros_msg_to_influx_point(
            msg=msg, measurement_name=measurement_name, tags=tags
        )
        client = DatabaseClient.get_instance()
        client.write(point)
        logger.info(f"Send: {measurement_name}")

    except Exception as e:
        logger.error(f"Failed to write {measurement_name}: {e}", exc_info=True)



# NOTE Handler example
# def cmd_vel_callback(msg: Twist, measurement_name: str, tags: dict[str, str] | None):
#     try:
#         point = ros_msg_to_influx_point(msg=msg, measurement_name="velocity", tags={})
#         client = DatabaseClient.get_instance()
#         client.write(point)
#         logger.info("Send cmd_vel")

#     except Exception as e:
#         logger.error(f"[CMD_VEL] ✗ Failed to write: {e}", exc_info=True)


def signal_state_callback(
    msg: SignalState, topic_name: str, tags: dict[str, str] | None
):
    """
    Custom handler for SignalState.
    Maps the uint8 'state' to a string tag/field for InfluxDB.
    """
    SIGNAL_STATE_MAP = {
        SignalState.IDLE: "IDLE",
        SignalState.GREETING: "GREETING",
        SignalState.SPEAKING: "SPEAKING",
        SignalState.MOVING: "MOVING",
        SignalState.STOPPED: "STOPPED",
        SignalState.REVERSE: "REVERSE",
        SignalState.ERROR_MINOR: "ERROR_MINOR",
        SignalState.ERROR_CRITICAL: "ERROR_CRITICAL",
        SignalState.LOW_BATTERY: "LOW_BATTERY",
    }
    measurement_name = "robot_signal_status"

    try:
        # Resolve the integer state to a string
        state_str = SIGNAL_STATE_MAP.get(msg.state, "UNKNOWN")

        # Create point manually to inject the mapped string
        point = Point(measurement_name)

        # Add base tags (e.g. robot_id)
        if tags:
            for k, v in tags.items():
                point.tag(k, v)

        # Add the mapped string as a TAG (better for filtering/grouping in Influx)
        point.tag("state_label", state_str)

        # Add the raw integer as a FIELD (better for visualizing state changes over time)
        point.field("state_code", int(msg.state))

        # Write to DB
        client = DatabaseClient.get_instance()
        client.write(point)
        logger.info(f"Send: {measurement_name} -> {state_str} ({msg.state})")

    except Exception as e:
        logger.error(f"Failed to write {measurement_name}: {e}", exc_info=True)


if __name__ == "__main__":
    setup_logger()
    _ = load_dotenv()
    mock = os.getenv("MOCK")
    if mock and mock.lower() == "true":
        main()
        try:
            _ = threading.Event().wait()
        except KeyboardInterrupt:
            print("Stopping...")
    else:
        node = rospy.init_node("turtlewatch", anonymous=True)
        main()
        rospy.spin()
