from fileinput import filelineno
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import logging
from bridge.database_client import DatabaseClient
from bridge.throttled_subscriber import ThrottledSubscriber
from bridge.utils import ros_msg_to_influx_point

# Setup Python logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("InfluxLogger")


def main():
    rospy.init_node("turtlewatch", anonymous=True)

    logger.info("Connecting to InfluxDB...")

    with open("../influxdb_token.txt", "r") as file:
        db_token = file.read().strip()

    DatabaseClient.intialize(
        host="http://localhost:8181", database="dev", token=db_token
    )
    logger.info("Successfully connected to InfluxDB (database: dev)")

    cmd_vel_sub = ThrottledSubscriber(
        topic_name="/cmd_vel",
        msg_class=Twist,
        callback=cmd_vel_callback,
        interval=rospy.Duration(1),
    )

    cmd_vel_sub = ThrottledSubscriber(
        topic_name="/odom",
        msg_class=Odometry,
        callback=odom_callback,
        interval=rospy.Duration(1),
    )



def cmd_vel_callback(msg: Twist):
    try:
        point = ros_msg_to_influx_point(msg=msg, measurement_name="velocity", tags={})
        logger.info(point)
        client = DatabaseClient.get_instance()
        client.write(point)

    except Exception as e:
        logger.error(f"[CMD_VEL] ✗ Failed to write: {e}", exc_info=True)


def odom_callback(msg: Odometry):
    """Write odometry data to InfluxDB"""
    try:
        point = ros_msg_to_influx_point(msg=msg, measurement_name="odometry", tags={})
        logger.info(point)
        client = DatabaseClient.get_instance()
        client.write(point)

    except Exception as e:
        logger.error(f"[CMD_VEL] ✗ Failed to write: {e}", exc_info=True)


if __name__ == "__main__":
    main()
    rospy.spin()
