from contextlib import contextmanager
import logging
import sqlite3
from influxdb_client_3 import InfluxDBClient3, write_client_options, WriteOptions

logger = logging.getLogger("BridgeLogger")

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
        logger.info("Successfully initialized to InfluxDB")

    @classmethod
    def get_instance(cls) -> InfluxDBClient3:
        if cls._client_instance is None:
            raise RuntimeError("Please Initialize the DatabaseClient first")

        return cls._client_instance


class StatsDB:
    _client_instance: InfluxDBClient3 | None = None

    @classmethod
    def intialize(cls, host: str, database: str, token: str) -> None:
        if cls._client_instance is not None:
            raise RuntimeError("Already initialized")

        cls._client_instance = InfluxDBClient3(
            host=host, database=database, token=token, 
        )
        logger.info("Successfully initialized to StatsDB")

    @classmethod
    def get_instance(cls) -> InfluxDBClient3:
        if cls._client_instance is None:
            raise RuntimeError("Please Initialize the DatabaseClient first")

        return cls._client_instance
