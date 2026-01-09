import os
import sys
import threading
from dotenv import load_dotenv
import genpy
import rospy
from bridge import plugin_loader
from bridge.publishers import register_publishers
from bridge.stats import StatsTracker
import logging
from bridge.database_client import InfluxDB, StatsDB
from bridge.throttled_subscriber import ThrottledSubscriber
from bridge.utils import ros_msg_to_influx_point

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

def main(active_plugins: list[plugin_loader.Plugin[genpy.Message]]):
    for plugin in active_plugins:
        _ = ThrottledSubscriber[genpy.Message](plugin=plugin)

if __name__ == "__main__":
    setup_logger()
    _ = load_dotenv()
    mock = os.getenv("MOCK")

    logger.info("Connecting to InfluxDB...")

    influxDB_token = os.getenv("INFLUXDB_TOKEN")
    if not influxDB_token:
        with open("../influxdb_token.txt", "r") as file:
            influxDB_token = file.read().strip()

    influxDB_name = os.getenv("INFLUXDB_METRICS_DB_NAME")
    if not influxDB_name:
        influxDB_name = "dev"

    influxDB_url = os.getenv("INFLUXDB_URL")
    if not influxDB_url:
        influxDB_url = "http://localhost:8181"
    InfluxDB.intialize(host=influxDB_url, database=influxDB_name, token=influxDB_token)

    stats_db_name = os.getenv("INFLUXDB_STATISTICS_DB_NAME")
    if not stats_db_name:
        stats_db_name = "sessions"
    StatsDB.intialize(host=influxDB_url, database=stats_db_name, token=influxDB_token)

    active_plugins = plugin_loader.load_plugins()

    StatsTracker.start_new_session()

    if mock and mock.lower() == "true":
        # MOCKING 
        main(active_plugins)
        try:
            _ = threading.Event().wait()
        except KeyboardInterrupt:
            print("Stopping...")
        # END MOCKING
    else:
        # PRODUCTION
        node = rospy.init_node("turtlewatch", anonymous=True)  # pyright: ignore [reportUnknownMemberType]
        register_publishers()
        main(active_plugins)
        rospy.spin()
        # END PRODUCTION
