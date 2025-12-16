from dataclasses import dataclass, field
from enum import Enum
import logging
import time
import uuid
from .database_client import InfluxDB, StatsDB
import pyarrow as pa

logger = logging.getLogger("BridgeLogger")


class CompletionStatus(Enum):
    PENDING = 0
    IN_PROGRESS = 1
    COMPLETED = 2
    FAILED = 3
    ABORTED = 4


@dataclass
class NavigationMetrics:
    total_distance_meters: float = 0
    avg_linear_velocity: float = 0
    max_linear_velocity: float = 0
    idle_time: float = 0

@dataclass
class Session:
    """One Session is one trip of the robot from his start point back to his start point"""
    id: uuid.UUID = field(default_factory=uuid.uuid4)
    start_time: int = field(default_factory=time.time_ns)
    end_time: int | None = None
    number_of_messages: int = 0
    completion_status: CompletionStatus = CompletionStatus.PENDING
    navigation_metrics: NavigationMetrics = field(default_factory=NavigationMetrics)


initialize_schema_query = """
CREATE TABLE IF NOT EXISTS sessions (
    id TEXT PRIMARY KEY,
    start_time INTEGER NOT NULL,
    end_time INTEGER,
    number_of_messages INTEGER,
    completion_status TEXT,
    total_distance_meters REAL,
    avg_linear_velocity REAL,
    max_linear_velocity REAL,
    idle_time REAL
);
"""


class StatsTracker:
    # NOTE just start a sample one on startup so we don't have to care about None type
    _current_session: Session = Session()

    @classmethod
    def initialize_db_schema(cls):
        try:
            StatsDB.execute(initialize_schema_query)
        except Exception as e:
            logger.error("Error intializing schema in statsDB", e)

    @classmethod
    def start_new_session(cls):
        cls._current_session = Session()

    @classmethod
    def stop_current_session(cls):
        cls._current_session.end_time = time.time_ns()
        cls._calculate_navigation_metrics()
        cls.update_stats_db()

    @classmethod
    def _calculate_navigation_metrics(cls):
        s = cls._current_session
        if s.start_time is None or s.end_time is None:
            logger.warning("Cannot calculate metrics without a session start and end time")
            return

        client = InfluxDB.get_instance()

        queries = {
            "total_distance_meters": f"""
                SELECT INTEGRAL(v, 1s) AS total_distance_meters
                FROM (
                    SELECT MEAN("linear_x") AS v
                    FROM "cmd_vel"
                    WHERE time >= {s.start_time} AND time <= {s.end_time}
                    GROUP BY time(1s)
                )
            """,
            "avg_linear_velocity": f"""
                SELECT MEAN("linear_x") AS avg_linear_velocity
                FROM "cmd_vel"
                WHERE time >= {s.start_time} AND time <= {s.end_time}
            """,
            "max_linear_velocity": f"""
                SELECT MAX("linear_x") AS max_linear_velocity
                FROM "cmd_vel"
                WHERE time >= {s.start_time} AND time <= {s.end_time}
            """,
        }

        try:
            for name, query in queries.items():
                table: pa.Table = client.query(
                    query=query,
                    language="influxql",
                    mode="pyarrow",
                )

                if table.num_rows > 0:
                    value = table.column(name)[0].as_py()
                    setattr(s.navigation_metrics, name, float(value))
        except Exception as e:
            logger.error(f"Error querying InfluxDB navigation metrics: {e}")

    @classmethod
    def update_stats_db(cls):
        query = """
            INSERT INTO sessions (
                id,
                start_time,
                end_time,
                number_of_messages,
                completion_status,
                total_distance_meters,
                avg_linear_velocity,
                max_linear_velocity,
                idle_time
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            ON CONFLICT(id) DO UPDATE SET
                end_time = excluded.end_time,
                number_of_messages = excluded.number_of_messages,
                completion_status = excluded.completion_status,
                total_distance_meters = excluded.total_distance_meters,
                avg_linear_velocity = excluded.avg_linear_velocity,
                max_linear_velocity = excluded.max_linear_velocity,
                idle_time = excluded.idle_time;
        """
        s = cls._current_session
        params = (
            str(s.id),
            s.start_time,
            s.end_time,
            s.number_of_messages,
            s.completion_status.name,
            s.navigation_metrics.total_distance_meters,
            s.navigation_metrics.avg_linear_velocity,
            s.navigation_metrics.max_linear_velocity,
            s.navigation_metrics.idle_time,
        )
        try:
            StatsDB.execute(query, params)
        except Exception as e:
            logger.error("Error writing to statsDB", e)

    @classmethod
    def get_session(cls) -> Session:
        return cls._current_session

