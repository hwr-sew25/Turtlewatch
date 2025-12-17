from dataclasses import dataclass, field
import logging
import time
import uuid

from ros_msgs.actionlib_msgs.msg._GoalStatus import GoalStatus
from .database_client import InfluxDB, StatsDB
import pandas as pd

logger = logging.getLogger("BridgeLogger")


# class CompletionStatus(Enum):
#     PENDING = 0
#     IN_PROGRESS = 1
#     COMPLETED = 2
#     FAILED = 3
#     ABORTED = 4


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
    # NOTE change this for now, dunno what the actual status will look like
    # completion_status: CompletionStatus = CompletionStatus.PENDING
    completion_status: int = (
        GoalStatus.PENDING
    )  # we need to use int here because the other thing is a literal
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
        import numpy as np

        s = cls._current_session
        if s.start_time is None or s.end_time is None:
            logger.warning(
                "Cannot calculate metrics without a session start and end time"
            )
            return

        client = InfluxDB.get_instance()

        try:
            # 1. Scalar Metrics (Avg, Max)
            # We can fetch these in one go or keep them separate. Keeping separate for clarity.
            scalar_queries = {
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

            for name, query in scalar_queries.items():
                # 'dataframe' mode returns a Pandas DataFrame, which wraps NumPy
                df: pd.DataFrame = client.query(
                    query=query, language="influxql", mode="pandas"
                )

                if not df.empty:
                    # .iloc[0] gets the first row's value
                    val = df[name].iloc[0]
                    setattr(s.navigation_metrics, name, float(val))

            dist_query = f"""
                SELECT "linear_x"
                FROM "cmd_vel"
                WHERE time >= {s.start_time} AND time <= {s.end_time}
            """

            df_dist: pd.DataFrame = client.query(
                query=dist_query, language="influxql", mode="pandas"
            )

            if not df_dist.empty and len(df_dist) > 1:
                # 1. Get velocity (y-axis)
                velocity = df_dist["linear_x"]

                # 2. Get time (x-axis)
                # Ensure 'time' is datetime, even if Influx returned strings
                timestamps = pd.to_datetime(df_dist["time"])

                # Convert to nanoseconds (int64)
                t_nanos = timestamps.astype(np.int64)

                # Normalize to seconds relative to the start (t - t0)
                # This gives us a clean time array: [0.0, 0.25, 0.50, 0.75 ...]
                t_seconds = (t_nanos - t_nanos.iloc[0]) / 1e9

                # 3. Calculate Integral
                # np.trapz(y, x) calculates the area under the curve
                distance = np.trapz(y=velocity, x=t_seconds)

                s.navigation_metrics.total_distance_meters = float(distance)
            else:
                s.navigation_metrics.total_distance_meters = 0.0
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
            s.completion_status,
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
