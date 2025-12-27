import logging
import threading
import time
from typing import Any

import genpy
from bridge.plugin_loader import Plugin

logger = logging.getLogger("BridgeLogger")


def mock_subscriber(plugin: Plugin[genpy.Message]) -> None:
    # NOTE it would be more efficient if these could be asyncio coroutines
    # instead of threads but I don'want to change everything to async since
    # that stuff if managed by ROS if we are not mocking
    # TODO this should probably be refactored to use threading.Event() instead of daemon threads
    thread = threading.Thread(target=dispatcher, args=[plugin], daemon=True)
    thread.start()


def dispatcher(
    plugin: Plugin[genpy.Message],
) -> None:
    iteration = 0
    start_time = time.time()
    state: dict[str, Any] = {}
    while True:
        (msg, state) = plugin.mock_generator(state)
        plugin.callback(msg)

        iteration += 1
        next_time = start_time + (iteration * plugin.interval)
        sleep_duration = next_time - time.time()
        if sleep_duration > 0:
            time.sleep(sleep_duration)
