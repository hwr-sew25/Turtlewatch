import os
import sys
import threading
from actionlib_msgs.msg import GoalStatusArray
from dotenv import load_dotenv
import genpy
from influxdb_client_3 import Point
import rospy
from bridge.stats import StatsTracker
from bridge.alert import AlertSystem
from ros_msgs.actionlib_msgs.msg._GoalStatus import GoalStatus
from ros_msgs.geometry_msgs.msg import Twist
import logging
from bridge.database_client import InfluxDB, InfluxDB, StatsDB
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
    topics_config = {
        # ros_topic_name: (MessageType, Callback function)
        "/cmd_vel": (Twist, generic_callback),
        "/odom": (Odometry, generic_callback),
        "/battery_state": (BatteryState, generic_callback),
        "/move_base/status": (GoalStatusArray, move_status_callback),
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
        client = influxdb.get_instance()
        client.write(point)
        logger.info(f"send: {measurement_name}")

    except Exception as e:
        logger.error(f"Failed to write {measurement_name}: {e}", exc_info=True)


def move_status_callback(msg: GoalStatus, topic_name: str, tags: dict[str, str] | None):
    """
    Custom handler for Move/GoalStatus.
    Maps the uint8 'status' to a string tag/field for InfluxDB.
    """
    measurement_name = topic_name.removeprefix("/").replace("/", "_")
    GOAL_STATUS_MAP = {
        GoalStatus.PENDING: "PENDING",
        GoalStatus.ACTIVE: "ACTIVE",
        GoalStatus.PREEMPTED: "PREEMPTED",
        GoalStatus.SUCCEEDED: "SUCCEEDED",
        GoalStatus.ABORTED: "ABORTED",
        GoalStatus.REJECTED: "REJECTED",
        GoalStatus.PREEMPTING: "PREEMPTING",
        GoalStatus.RECALLING: "RECALLING",
        GoalStatus.RECALLED: "RECALLED",
        GoalStatus.LOST: "LOST",
    }

    session = StatsTracker.get_session()
    session.completion_status = msg.status
    logger.info(f"Received goal status: {msg.status}")

    if msg.status in [
        GoalStatus.ABORTED,
        GoalStatus.SUCCEEDED,
        GoalStatus.RECALLED,
        GoalStatus.LOST,
    ]:
        StatsTracker.stop_current_session()

    try:
        # Get the string representation, default to UNKNOWN if status ID is invalid
        state_str = GOAL_STATUS_MAP.get(msg.status, "UNKNOWN")

        point = Point(measurement_name)

        # Apply existing tags
        if tags:
            for k, v in tags.items():
                point.tag(k, v)

        # Add specific status data
        point.tag("state_label", state_str)
        point.field("state_code", int(msg.status))

        client = InfluxDB.get_instance()
        client.write(point)
        logger.info(f"Send: {measurement_name} -> {state_str} ({msg.status})")

    except Exception as e:
        logger.error(f"Failed to write {measurement_name}: {e}", exc_info=True)


def signal_state_callback(
    msg: SignalState, topic_name: str, tags: dict[str, str] | None
):
    """
    Custom handler for SignalState.
    Maps the uint8 'state' to a string tag/field for InfluxDB.
    """
    measurement_name = topic_name.removeprefix("/").replace("/", "_")
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

    if msg.state == SignalState.ERROR_CRITICAL:
        AlertSystem.send_slack_message("C09PRS9P08K", "CRITICAL ERROR")

    try:
        state_str = SIGNAL_STATE_MAP.get(msg.state, "UNKNOWN")

        point = Point(measurement_name)

        if tags:
            for k, v in tags.items():
                point.tag(k, v)

        point.tag("state_label", state_str)

        point.field("state_code", int(msg.state))

        client = InfluxDB.get_instance()
        client.write(point)
        logger.info(f"Send: {measurement_name} -> {state_str} ({msg.state})")

    except Exception as e:
        logger.error(f"Failed to write {measurement_name}: {e}", exc_info=True)


if __name__ == "__main__":
    setup_logger()
    _ = load_dotenv()
    mock = os.getenv("MOCK")

    logger.info("Connecting to InfluxDB...")

    influxDB_token = os.getenv("INFLUXDB_TOKEN")
    if not influxDB_token:
        with open("../influxdb_token.txt", "r") as file:
            influxDB_token = file.read().strip()

    influxDB_name = os.getenv("INFLUXDB_DB_NAME")
    if not influxDB_name:
        influxDB_name = "dev"

    influxDB_url = os.getenv("INFLUXDB_URL")
    if not influxDB_url:
        influxDB_url = "http://localhost:8181"
    InfluxDB.intialize(host=influxDB_url, database=influxDB_name, token=influxDB_token)

    stats_db_name = os.getenv("INFLUXDB_DB_SESSIONS_NAME")
    if not stats_db_name:
       stats_db_name = "sessions"
    StatsDB.intialize(host=influxDB_url, database=stats_db_name, token=influxDB_token)

    StatsTracker.start_new_session()

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
