import logging

from influxdb_client_3 import InfluxDBClient3
from .database_client import InfluxDB, StatsDB

logger = logging.getLogger("BridgeLogger")

class StatsTracker:
    _influxdb: InfluxDBClient3 | None = None
    _statsdb = None
    number_of_messages: int = 0

    @classmethod
    def initialize(cls):
        cls._influxdb = InfluxDB.get_instance()
        cls._statsdb = StatsDB.get_connection()

    @classmethod
    def start_statistic_tracking(cls):
        while True:
            cls.update_stats_db()

    @classmethod
    def update_stats_db(cls):
       pass 

    @classmethod
    def update_number_of_messages(cls, amount: int):
        cls.number_of_messages += amount
