from dataclasses import dataclass
from enum import Enum
import logging
import time
import uuid
from .database_client import  StatsDB

logger = logging.getLogger("BridgeLogger")


class CompletionStatus(Enum):
    PENDING = 0
    IN_PROGRESS = 1
    COMPLETED = 2
    FAILED = 3 
    ABORTED = 4

@dataclass
class NavigationMetrics():
    total_distance_meters: float = 0
    avg_linear_velocity: float = 0
    max_linear_velocity: float = 0
    idle_time: float = 0

class Session:
    """ One Session is one trip of the robot from his start point back to his start point """
    id: uuid.UUID = uuid.uuid4()
    start_time: int 
    end_time: int = 0
    number_of_messages: int = 0
    completion_status: CompletionStatus = CompletionStatus.PENDING
    navigation_metrics: NavigationMetrics = NavigationMetrics()

    def __init__(self):
        self.start_time = time.time_ns()
        self.completion_status = CompletionStatus.IN_PROGRESS

    def stop(self):
        self.end_time = time.time_ns()


class StatsTracker:
    # NOTE just start a sample one on startup so we don't have to care about None type
    _current_session:Session =  Session()

    @classmethod
    def initialize_db_schema(cls):
        def update_stats_db(cls):
            StatsDB.execute("")

    @classmethod
    def start_new_session(cls):
        cls._current_session = Session()

    @classmethod
    def stop_current_session(cls):
        cls._current_session.stop()

    # TODO think about not tracking everything in the session but gbet the data out of
    # influxDB based on the start time
    @classmethod
    def update_stats_db(cls):
        StatsDB.execute("")

    @classmethod
    def update_number_of_messages(cls, amount: int):
        cls._current_session.number_of_messages += amount

