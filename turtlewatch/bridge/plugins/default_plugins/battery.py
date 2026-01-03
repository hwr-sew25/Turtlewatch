import time
import genpy
from typing import Any, override
from bridge.plugin_loader import Plugin
from sensor_msgs.msg._BatteryState import BatteryState
from std_msgs.msg._Header import Header


class BatteryPlugin(Plugin[BatteryState]):
    topic_name: str = "/battery_state"
    ros_msg_type: type[BatteryState] = BatteryState
    is_enabled: bool = True
    tags: dict[str, str] = {}
    interval: float = 0.25

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
        if state["percentage"] <= 0.0:
            state["percentage"] = 1.0
            state["voltage"] = 12.3

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
