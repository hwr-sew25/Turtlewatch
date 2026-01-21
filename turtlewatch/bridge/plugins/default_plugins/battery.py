import os
import time
import genpy
from typing import Any, override

from bridge.plugin_loader import Plugin
from bridge.publishers import Publisher, get_publisher
from sensor_msgs.msg._BatteryState import BatteryState
from std_msgs.msg._Header import Header


class BatteryPlugin(Plugin[BatteryState]):
    topic_name: str = "/battery_state"
    ros_msg_type: type[BatteryState] = BatteryState
    is_enabled: bool = True
    tags: dict[str, str] = {}
    interval: float = 0.25


## neue battery logik (volt to percent)
    _voltage_min: float = 11.0
    _voltage_max: float = 12.3
    _voltage_range: float = _voltage_max - _voltage_min

    def _calculate_percent(self, voltage: float) -> int:
        raw_percent = ((voltage - self._voltage_min) / self._voltage_range) * 100
        return max(0, min(100, int(raw_percent)))

    @override
    def callback(self, msg: BatteryState) -> None:
        voltage = msg.voltage
        if voltage is None:
            return

        percent_value = self._calculate_percent(voltage)
        msg.percentage = percent_value / 100.0

        mock = os.getenv("MOCK")
        if not mock or mock.lower() != "true":
            publisher = get_publisher(Publisher.BATTERY_STATE)
            if publisher:
                publisher.publish(msg)

        super().callback(msg)

    @override
    def mock_generator(
        self, state: dict[str, Any]
    ) -> tuple[BatteryState, dict[str, Any]]:
        """
        Simulates a battery draining over time.
        Returns sensor_msgs/BatteryState.
        """

        # 1. Initialize State if it's the first run
        if not state:
            state = {
                "percentage": 1.0,  # 1.0 = 100%
                "voltage": 12.3,  # Typical full charge for a 3S LiPo
            }

        msg = BatteryState()
        msg.header = Header()
        msg.header.stamp = genpy.Time.from_sec(time.time())  # pyright: ignore [reportUnknownMemberType]

        # 2. Simulate Draining
        # Decrease percentage by 0.5% every callback
        state["percentage"] -= 0.005
        # Simulate voltage drop roughly proportional to percentage
        state["voltage"] -= 0.01

        # 3. Reset logic (infinite loop simulation)
        percent_value = self._calculate_percent(state["voltage"])
        state["percentage"] = percent_value / 100.0
        if percent_value <= 0:
            state["percentage"] = 1.0
            state["voltage"] = self._voltage_max

        # 4. Populate Message Fields
        msg.voltage = state["voltage"]
        msg.percentage = state["percentage"]
        msg.current = -1.5  # Negative current indicates discharging
        msg.charge = 4.0  # Ah current capacity
        msg.capacity = 4.0  # Ah total capacity
        msg.design_capacity = 4.0

        # Use standard constants from the message definition
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True

        return (msg, state)
