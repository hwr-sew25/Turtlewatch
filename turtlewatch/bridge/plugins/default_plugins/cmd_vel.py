import time
from typing import Any, override
from geometry_msgs.msg import Twist

from bridge.plugin_loader import Plugin


class CmdVelPlugin(Plugin[Twist]):
    topic_name: str = "/cmd_vel"
    ros_msg_type: type[Twist] = Twist
    is_enabled: bool = True
    interval: float = 0.25

    # Predefined route: list of (duration_seconds, linear_x, angular_z, description)
    # linear_x: forward speed (m/s)
    # angular_z: rotation speed (rad/s), positive = left turn
    PREDEFINED_ROUTE: list[tuple[float, float, float, str]] = [
        (8.0, 0.8, 0.0, "straight"),  # Start: drive straight (fast, long distance)
        (1.0, 0.0, 0.0, "stop"),  # Stop
        (2.0, 0.0, 0.8, "turn_left"),  # Turn left 90 deg
        (6.0, 0.8, 0.0, "straight"),  # Drive straight
        (1.0, 0.0, 0.0, "stop"),  # Stop
        (10.0, 0.5, 0.4, "circle"),  # Drive in large circle (forward + turning)
        (1.0, 0.0, 0.0, "stop"),  # Stop
        (2.0, 0.0, -0.8, "turn_right"),  # Turn right 90 deg
        (10.0, 0.8, 0.0, "straight"),  # Drive straight (long)
        (1.0, 0.0, 0.0, "stop"),  # Stop
        (4.0, -0.5, 0.0, "backward"),  # Drive backwards
        (1.0, 0.0, 0.0, "stop"),  # Stop
        (8.0, 0.4, -0.5, "circle_right"),  # Large circle other direction
        (1.0, 0.0, 0.0, "stop"),  # Stop
        (6.0, 0.6, 0.2, "curve_back"),  # Curve back towards center
        (2.0, 0.0, 0.0, "stop"),  # Final stop
    ]

    def _get_route_command(self, elapsed_time: float) -> tuple[float, float, str]:
        """Helper to get the current velocity command based on elapsed time."""
        total_duration = sum(cmd[0] for cmd in self.PREDEFINED_ROUTE)

        # Loop the route
        loop_time = elapsed_time % total_duration
        current_time_cursor = 0.0

        for duration, linear_x, angular_z, desc in self.PREDEFINED_ROUTE:
            if current_time_cursor + duration > loop_time:
                return (linear_x, angular_z, desc)
            current_time_cursor += duration

        return (0.0, 0.0, "stop")

    @override
    def mock_generator(self, state: dict[str, Any]) -> tuple[Twist, dict[str, Any]]:
        # Initialize state if this is the first run
        if not state:
            state = {"start_time": time.time()}

        # Calculate elapsed time
        elapsed = time.time() - state["start_time"]

        # Get command from helper
        linear_x, angular_z, _ = self._get_route_command(elapsed)

        # Construct ROS message
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular_z

        return (msg, state)
