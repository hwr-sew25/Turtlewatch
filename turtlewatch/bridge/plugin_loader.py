from __future__ import annotations
from abc import ABC, abstractmethod
import importlib
import inspect
import logging
import pkgutil
from typing import Any
import genpy

from bridge.database_client import InfluxDB
from bridge.utils import ros_msg_to_influx_point

logger = logging.getLogger("BridgeLogger")
active_plugins: list[Plugin[genpy.Message]] = []


class Plugin[MsgType: genpy.Message](ABC):
    """
    Base class for all Plugins.
    Authors should inherit from this and implement at least `mock_generator`.
    """

    ros_msg_type: type[MsgType]
    topic_name: str

    is_enabled: bool = True
    tags: dict[str, str] = {}
    interval: float = 0.25

    def log(self, message: str, level: int = logging.INFO) -> None:
        """
        Helper for plugins to log messages. Adds the topic name.
        """
        logger.log(level, f"[{self.topic_name}] {message}")

    def callback(self, msg: MsgType) -> None:
        """
        Default implementation for sending data to InfluxDB.
        Can be overridden if custom processing is needed.
        """
        measurement_name = self.topic_name.removeprefix("/").replace("/", "_")

        try:
            point = ros_msg_to_influx_point(
                msg=msg, measurement_name=measurement_name, tags=self.tags
            )
            client = InfluxDB.get_instance()
            client.write(point) # pyright: ignore [reportUnknownMemberType]

            logger.info(f"[{self.topic_name}] Sent measurement: {measurement_name}")

        except Exception as e:
            logger.error(
                f"[{self.topic_name}] Failed to write {measurement_name}: {e}",
                exc_info=True,
            )

    @abstractmethod
    def mock_generator(self, state: dict[str, Any]) -> tuple[MsgType, dict[str, Any]]:
        """
        Generate a mock message and update the state.
        Must be implemented by the plugin author.
        """
        pass


def load_plugins():
    """
    Scans 'default_plugins' and 'custom_plugins' for any class inheriting from Plugin.
    """

    # We explicitly define which sub-folders to scan
    # This prevents the loader from getting lost in __init__.py files
    plugin_namespaces = [
        "bridge.plugins.default_plugins",
        "bridge.plugins.custom_plugins",
    ]

    for namespace in plugin_namespaces:
        try:
            # 1. Import the folder as a package
            package = importlib.import_module(namespace)
        except ImportError as e:
            logger.warning(f"Could not check plugin folder '{namespace}': {e}")
            continue

        # 2. Iterate over the actual .py files inside (e.g., cmd_vel.py)
        for _, module_name, _ in pkgutil.iter_modules(package.__path__):
            # Construct the full dot-path: bridge.plugins.default_plugins.cmd_vel
            full_module_name = f"{namespace}.{module_name}"

            try:
                module = importlib.import_module(full_module_name)

                # 3. Inspect for valid Plugin classes
                for _ , obj in inspect.getmembers(module, inspect.isclass):
                    # Must inherit from Plugin, but NOT be the Plugin class itself
                    if issubclass(obj, Plugin) and obj is not Plugin:
                        # Instantiate the plugin
                        instance: Plugin[genpy.Message] = obj() # pyright: ignore [reportUnknownVariableType]

                        if instance.is_enabled:
                            logger.info(
                                f"Loaded plugin: {instance.topic_name} ({full_module_name})"
                            )
                            active_plugins.append(instance)

            except Exception as e:
                logger.error(f"Failed to load {full_module_name}: {e}", exc_info=True)
    return active_plugins
