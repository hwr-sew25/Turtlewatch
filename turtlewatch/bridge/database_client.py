from contextlib import contextmanager
import sqlite3
from influxdb_client_3 import InfluxDBClient3, write_client_options, WriteOptions


class InfluxDB:
    _client_instance: InfluxDBClient3 | None = None

    @classmethod
    def intialize(cls, host: str, database: str, token: str) -> None:
        if cls._client_instance is not None:
            raise RuntimeError("Already initialized")

        write_options = WriteOptions(batch_size=4)
        wco = write_client_options(write_options=write_options)

        cls._client_instance = InfluxDBClient3(
            host=host, database=database, token=token, write_client_options=wco
        )

    @classmethod
    def get_instance(cls) -> InfluxDBClient3:
        if cls._client_instance is None:
            raise RuntimeError("Please Initialize the DatabaseClient first")

        return cls._client_instance


class StatsDB:
    """
    A static helper for managing a single SQLite database connection.
    Usage:
        1. Call StatsDB.initialize("db_name") at startup.
        2. Use StatsDB.execute() or StatsDB.query() anywhere.
    """

    _db_name: str | None = None

    @classmethod
    def initialize(cls, db_name: str):
        """
        Sets the database path and verifies the connection immediately.
        """
        if cls._db_name is not None:
            raise RuntimeError("Already initialized")
        cls._db_name = db_name
        try:
            with cls.get_connection() as conn:
                conn.execute("SELECT 1")
        except Exception as e:
            cls._db_name = None
            raise RuntimeError(f"Could not connect to database '{db_name}': {e}")

    @classmethod
    @contextmanager
    def get_connection(cls):
        """
        Context manager that yields a raw sqlite3 connection.
        Handles opening and closing the connection automatically.

        Example:
            with StatsDB.get_connection() as conn:
                cursor = conn.cursor()
                # ... custom work ...
        """
        if cls._db_name is None:
            raise RuntimeError("Run StatsDB.initialize() first")

        conn = sqlite3.connect(cls._db_name)
        conn.row_factory = sqlite3.Row
        try:
            yield conn
        finally:
            conn.close()

    @classmethod
    def execute(cls, sql: str, params=()):
        """
        Runs a write operation (INSERT, UPDATE, DELETE).
        Automatically commits changes.

        Returns:
            int: The ID of the last inserted row (if applicable).

        Example:
            user_id = StatsDB.execute("INSERT INTO users (name) VALUES (?)", ("Alice",))
        """
        with cls.get_connection() as conn:
            with conn:  # Auto-commit
                cur = conn.cursor()
                cur.execute(sql, params)
                return cur.lastrowid

    @classmethod
    def query(cls, sql: str, params=()):
        """
        Runs a read operation (SELECT).

        Returns:
            list: A list of row objects (access columns like dicts: row['id']).

        Example:
            users = StatsDB.query("SELECT * FROM users WHERE active = ?", (1,))
            for user in users:
                print(user['name'])
        """
        with cls.get_connection() as conn:
            cur = conn.cursor()
            cur.execute(sql, params)
            return cur.fetchall()
