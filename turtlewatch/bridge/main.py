import os
import sys
import threading
from typing import Callable
import genpy
import rospy
from geometry_msgs.msg import Twist
import logging
from bridge.database_client import DatabaseClient
from bridge.throttled_subscriber import ThrottledSubscriber
from bridge.utils import ros_msg_to_influx_point
from bridge.types import Seconds

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

    topics: dict[str, Callable[[genpy.Message, str, dict[str, str] | None], None]] = {
        "/cmd_vel": generic_callback,
        "/odom": generic_callback,
    }

    for topic_name, callback_handler in topics.items():
        cmd_vel_sub = ThrottledSubscriber(
            topic_name=topic_name,
            msg_class=Twist,
            callback=callback_handler,
            interval=Seconds(0.25),
        )


def generic_callback(
    msg: genpy.Message, topic_name: str, tags: dict[str, str] | None
):
    measurement_name = topic_name.removeprefix("/")
    try:
        point = ros_msg_to_influx_point(
            msg=msg, measurement_name=measurement_name, tags=tags
        )
        client = DatabaseClient.get_instance()
        client.write(point)
        logger.info(f"Send: {msg}")

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


if __name__ == "__main__":
    setup_logger()
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
