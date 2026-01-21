from typing import Any, override

from bridge.plugin_loader import Plugin
from std_msgs.msg._Bool import Bool
from std_msgs.msg._String import String


class CurrentScreenPlugin(Plugin[String]):
    topic_name: str = "/current_screen"
    ros_msg_type: type[String] = String
    is_enabled: bool = True
    tags: dict[str, str] = {}
    interval: float = 0.25

    @override
    def mock_generator(self, state: dict[str, Any]) -> tuple[String, dict[str, Any]]:
        if not state:
            state = {
                "index": 0,
                "screens": [
                    "start",
                    "raumwahl",
                    "karte",
                    "kaffeeautomat",
                    "muelleimer",
                    "bildungsangebote",
                    "karten_ausgabe",
                    "snackautomat",
                    "spendenbox",
                    "geschichte_hwr",
                    "error1",
                    "error2",
                ],
            }

        state["index"] = (state["index"] + 1) % len(state["screens"])

        msg = String()
        msg.data = state["screens"][state["index"]]

        return (msg, state)


class LanguagePlugin(Plugin[String]):
    topic_name: str = "/language"
    ros_msg_type: type[String] = String
    is_enabled: bool = True
    tags: dict[str, str] = {}
    interval: float = 0.25

    @override
    def mock_generator(self, state: dict[str, Any]) -> tuple[String, dict[str, Any]]:
        if not state:
            state = {"index": 0, "languages": ["de", "en"]}

        state["index"] = (state["index"] + 1) % len(state["languages"])

        msg = String()
        msg.data = state["languages"][state["index"]]

        return (msg, state)


class StartDisplayPlugin(Plugin[Bool]):
    topic_name: str = "/display/start_druecken"
    ros_msg_type: type[Bool] = Bool
    is_enabled: bool = True
    tags: dict[str, str] = {}
    interval: float = 0.25

    @override
    def mock_generator(self, state: dict[str, Any]) -> tuple[Bool, dict[str, Any]]:
        if not state:
            state = {"counter": 0, "toggle_after": 6}

        state["counter"] += 1
        msg = Bool()
        msg.data = (state["counter"] // state["toggle_after"]) % 2 == 1

        return (msg, state)


class StopDisplayPlugin(Plugin[Bool]):
    topic_name: str = "/display/stop_druecken"
    ros_msg_type: type[Bool] = Bool
    is_enabled: bool = True
    tags: dict[str, str] = {}
    interval: float = 0.25

    @override
    def mock_generator(self, state: dict[str, Any]) -> tuple[Bool, dict[str, Any]]:
        if not state:
            state = {"counter": 0, "toggle_after": 8}

        state["counter"] += 1
        msg = Bool()
        msg.data = (state["counter"] // state["toggle_after"]) % 2 == 1

        return (msg, state)
