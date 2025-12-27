import math
import time
import genpy
from typing import Any, override
from bridge.plugin_loader import Plugin
from nav_msgs.msg._Odometry import Odometry
from std_msgs.msg._Header import Header


class OdometryPlugin(Plugin[Odometry]):
    topic_name: str = "/odom"
    ros_msg_type: type[Odometry] = Odometry
    is_enabled: bool = True
    tags: dict[str, str] = {}
    interval: float = 0.25

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
    def mock_generator(self, state: dict[str, Any]) -> tuple[Odometry, dict[str, Any]]:
        """Calculates position based on the predefined route commands.

        Integrates velocity commands over time to compute position.
        """

        if not state:
            state = {
                "x_pos": 0.0,
                "y_pos": 0.0,
                "theta": 0.0,
                "last_time": time.time(),
                "start_time": time.time(),
            }
        msg = Odometry()
        msg.header = Header()
        msg.header.stamp = genpy.Time.from_sec(time.time()) # pyright: ignore [reportUnknownMemberType]
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        # Calculate time delta
        current_time = time.time()
        dt = current_time - state["last_time"]
        state["last_time"] = current_time

        # Get current velocity command
        elapsed = current_time - state["start_time"]
        linear_x, angular_z, _ = self._get_route_command(elapsed)

        # Update heading (integrate angular velocity)
        state["theta"] += angular_z * dt

        # Update position (integrate linear velocity)
        state["x_pos"] += linear_x * math.cos(state["theta"]) * dt
        state["y_pos"] += linear_x * math.sin(state["theta"]) * dt

        msg.pose.pose.position.x = state["x_pos"]
        msg.pose.pose.position.y = state["y_pos"]
        msg.pose.pose.position.z = 0.0

        # Convert heading to quaternion
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(state["theta"] / 2)
        msg.pose.pose.orientation.w = math.cos(state["theta"] / 2)

        # Current velocity
        msg.twist.twist.linear.x = linear_x
        msg.twist.twist.angular.z = angular_z

        return (msg, state)
