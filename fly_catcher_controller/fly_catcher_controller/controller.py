#!/usr/bin/env python3
"""ROS 2 controller that intercepts a moving fly with a simulated drone."""

import math
from typing import Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from fly_catcher_interfaces.msg import DroneCmd
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

Vector3 = Tuple[float, float, float]


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp value to an inclusive range."""
    return max(lower, min(upper, value))


def norm(vector: Vector3) -> float:
    """Return the Euclidean length of a 3D vector."""
    return math.sqrt(sum(component * component for component in vector))


def subtract(a: Vector3, b: Vector3) -> Vector3:
    """Return vector a - b."""
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def add_scaled(position: Vector3, velocity: Vector3, scale: float) -> Vector3:
    """Return position + velocity * scale."""
    return (
        position[0] + velocity[0] * scale,
        position[1] + velocity[1] * scale,
        position[2] + velocity[2] * scale,
    )


def limit_norm(vector: Vector3, max_length: float) -> Vector3:
    """Limit a vector's magnitude while preserving direction."""
    length = norm(vector)
    if length <= max_length or length < 1e-9:
        return vector
    scale = max_length / length
    return (vector[0] * scale, vector[1] * scale, vector[2] * scale)


class FlyCatcherController(Node):
    """Estimate fly motion, calculate an intercept point, and command the drone."""

    def __init__(self) -> None:
        super().__init__("fly_catcher_controller")

        # Topic and frame parameters.
        self.declare_parameter("fly_topic", "/fly/position")
        self.declare_parameter("drone_state_topic", "/drone/state")
        self.declare_parameter("drone_cmd_topic", "/drone/cmd")
        self.declare_parameter("marker_topic", "/fly_catcher/markers")
        self.declare_parameter("world_frame", "map")

        # Controller parameters.
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("catch_distance_m", 0.2)
        self.declare_parameter("fly_timeout_s", 0.5)
        self.declare_parameter("drone_timeout_s", 0.5)
        self.declare_parameter("drone_speed_max_mps", 3.0)
        self.declare_parameter("chase_speed_max_mps", 3.0)
        self.declare_parameter("chase_speed_min_mps", 1.5)
        self.declare_parameter("slow_radius_m", 0.7)
        self.declare_parameter("lead_factor", 0.8)
        self.declare_parameter("max_prediction_time_s", 2.0)
        self.declare_parameter("velocity_filter_alpha", 0.5)
        self.declare_parameter("fly_velocity_limit_mps", 1.2)
        self.declare_parameter("status_rate_hz", 1.0)
        self.declare_parameter("marker_rate_hz", 10.0)

        self.fly_topic = str(self.get_parameter("fly_topic").value)
        self.drone_state_topic = str(
            self.get_parameter("drone_state_topic").value
        )
        self.drone_cmd_topic = str(self.get_parameter("drone_cmd_topic").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.world_frame = str(self.get_parameter("world_frame").value)

        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.catch_distance_m = float(
            self.get_parameter("catch_distance_m").value
        )
        self.fly_timeout_s = float(self.get_parameter("fly_timeout_s").value)
        self.drone_timeout_s = float(
            self.get_parameter("drone_timeout_s").value
        )
        self.drone_speed_max_mps = float(
            self.get_parameter("drone_speed_max_mps").value
        )
        self.chase_speed_max_mps = float(
            self.get_parameter("chase_speed_max_mps").value
        )
        self.chase_speed_min_mps = float(
            self.get_parameter("chase_speed_min_mps").value
        )
        self.slow_radius_m = float(self.get_parameter("slow_radius_m").value)
        self.lead_factor = float(self.get_parameter("lead_factor").value)
        self.max_prediction_time_s = float(
            self.get_parameter("max_prediction_time_s").value
        )
        self.velocity_filter_alpha = float(
            self.get_parameter("velocity_filter_alpha").value
        )
        self.fly_velocity_limit_mps = float(
            self.get_parameter("fly_velocity_limit_mps").value
        )
        status_rate_hz = float(self.get_parameter("status_rate_hz").value)
        marker_rate_hz = float(self.get_parameter("marker_rate_hz").value)

        self._validate_parameters()

        self.fly_position: Optional[Vector3] = None
        self.previous_fly_position: Optional[Vector3] = None
        self.fly_velocity: Vector3 = (0.0, 0.0, 0.0)
        self.previous_fly_stamp_s: Optional[float] = None
        self.last_fly_receive_s: Optional[float] = None

        self.drone_position: Optional[Vector3] = None
        self.drone_velocity: Vector3 = (0.0, 0.0, 0.0)
        self.last_drone_receive_s: Optional[float] = None

        self.intercept_point: Optional[Vector3] = None
        self.current_distance_m: Optional[float] = None
        self.caught = False
        self.start_time_s = self._now_s()
        self.catch_time_s: Optional[float] = None
        self.last_safety_log_s = -math.inf

        self.fly_subscription = self.create_subscription(
            PointStamped,
            self.fly_topic,
            self._on_fly_position,
            10,
        )
        self.drone_subscription = self.create_subscription(
            Odometry,
            self.drone_state_topic,
            self._on_drone_state,
            10,
        )
        self.command_publisher = self.create_publisher(
            DroneCmd,
            self.drone_cmd_topic,
            10,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            10,
        )

        self.control_timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self._control_step,
        )
        self.status_timer = self.create_timer(
            1.0 / max(status_rate_hz, 1e-3),
            self._publish_status,
        )
        self.marker_timer = self.create_timer(
            1.0 / max(marker_rate_hz, 1e-3),
            self._publish_markers,
        )

        self.get_logger().info(
            "Fly catcher controller started: "
            f"control={self.control_rate_hz:.1f} Hz, "
            f"catch_distance={self.catch_distance_m:.2f} m, "
            f"lead_factor={self.lead_factor:.2f}"
        )

    def _validate_parameters(self) -> None:
        """Fail early on invalid tuning values."""
        if self.control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be positive")
        if self.catch_distance_m <= 0.0:
            raise ValueError("catch_distance_m must be positive")
        if self.drone_speed_max_mps <= 0.0:
            raise ValueError("drone_speed_max_mps must be positive")
        if not 0.0 <= self.velocity_filter_alpha <= 1.0:
            raise ValueError("velocity_filter_alpha must be in [0, 1]")
        self.chase_speed_max_mps = clamp(
            self.chase_speed_max_mps,
            0.0,
            self.drone_speed_max_mps,
        )
        self.chase_speed_min_mps = clamp(
            self.chase_speed_min_mps,
            0.0,
            self.chase_speed_max_mps,
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_to_s(msg: PointStamped) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def _on_fly_position(self, msg: PointStamped) -> None:
        """Store position and estimate fly velocity from consecutive samples."""
        position = (float(msg.point.x), float(msg.point.y), float(msg.point.z))
        stamp_s = self._stamp_to_s(msg)
        receive_s = self._now_s()

        if self.previous_fly_position is not None and self.previous_fly_stamp_s is not None:
            dt = stamp_s - self.previous_fly_stamp_s
            if 1e-3 < dt < 1.0:
                raw_velocity = (
                    (position[0] - self.previous_fly_position[0]) / dt,
                    (position[1] - self.previous_fly_position[1]) / dt,
                    (position[2] - self.previous_fly_position[2]) / dt,
                )
                raw_velocity = limit_norm(
                    raw_velocity,
                    self.fly_velocity_limit_mps,
                )
                alpha = self.velocity_filter_alpha
                self.fly_velocity = (
                    alpha * raw_velocity[0] + (1.0 - alpha) * self.fly_velocity[0],
                    alpha * raw_velocity[1] + (1.0 - alpha) * self.fly_velocity[1],
                    alpha * raw_velocity[2] + (1.0 - alpha) * self.fly_velocity[2],
                )

        self.previous_fly_position = position
        self.previous_fly_stamp_s = stamp_s
        self.fly_position = position
        self.last_fly_receive_s = receive_s

    def _on_drone_state(self, msg: Odometry) -> None:
        """Store the latest drone position and velocity."""
        position = msg.pose.pose.position
        velocity = msg.twist.twist.linear
        self.drone_position = (
            float(position.x),
            float(position.y),
            float(position.z),
        )
        self.drone_velocity = (
            float(velocity.x),
            float(velocity.y),
            float(velocity.z),
        )
        self.last_drone_receive_s = self._now_s()

    def _measurements_are_fresh(self, now_s: float) -> bool:
        """Return False and stop safely when input data is missing or stale."""
        if (
            self.fly_position is None
            or self.drone_position is None
            or self.last_fly_receive_s is None
            or self.last_drone_receive_s is None
        ):
            self._log_safety("Waiting for fly and drone measurements", now_s)
            return False

        fly_age = now_s - self.last_fly_receive_s
        drone_age = now_s - self.last_drone_receive_s
        if fly_age > self.fly_timeout_s:
            self._log_safety(
                f"Fly measurement stale ({fly_age:.2f} s); commanding stop",
                now_s,
            )
            return False
        if drone_age > self.drone_timeout_s:
            self._log_safety(
                f"Drone state stale ({drone_age:.2f} s); commanding stop",
                now_s,
            )
            return False
        return True

    def _log_safety(self, message: str, now_s: float) -> None:
        """Rate-limit repeated safety messages."""
        if now_s - self.last_safety_log_s >= 1.0:
            self.get_logger().warn(message)
            self.last_safety_log_s = now_s

    def _calculate_intercept_time(
        self,
        relative_position: Vector3,
        fly_velocity: Vector3,
        drone_speed: float,
    ) -> float:
        """Solve ||r + v*t|| = drone_speed*t for the earliest positive t."""
        speed_squared = drone_speed * drone_speed
        fly_speed_squared = sum(component * component for component in fly_velocity)
        a = fly_speed_squared - speed_squared
        b = 2.0 * sum(
            relative_position[index] * fly_velocity[index]
            for index in range(3)
        )
        c = sum(component * component for component in relative_position)

        candidates = []
        epsilon = 1e-9
        if abs(a) < epsilon:
            if abs(b) >= epsilon:
                candidate = -c / b
                if candidate > 0.0:
                    candidates.append(candidate)
        else:
            discriminant = b * b - 4.0 * a * c
            if discriminant >= 0.0:
                root = math.sqrt(discriminant)
                first = (-b - root) / (2.0 * a)
                second = (-b + root) / (2.0 * a)
                if first > 0.0:
                    candidates.append(first)
                if second > 0.0:
                    candidates.append(second)

        if candidates:
            intercept_time = min(candidates)
        else:
            # Direct-pursuit fallback when no constant-velocity solution exists.
            intercept_time = math.sqrt(c) / max(drone_speed, 1e-6)

        return clamp(
            intercept_time * self.lead_factor,
            0.0,
            self.max_prediction_time_s,
        )

    def _desired_speed(self, distance_m: float) -> float:
        """Use maximum speed far away and slow slightly close to the target."""
        if self.slow_radius_m <= 1e-6 or distance_m >= self.slow_radius_m:
            return self.chase_speed_max_mps
        ratio = clamp(distance_m / self.slow_radius_m, 0.0, 1.0)
        return self.chase_speed_min_mps + ratio * (
            self.chase_speed_max_mps - self.chase_speed_min_mps
        )

    def _control_step(self) -> None:
        """Run the closed-loop lead/intercept controller."""
        now_s = self._now_s()
        if self.caught:
            self._publish_stop()
            return
        if not self._measurements_are_fresh(now_s):
            self._publish_stop()
            return

        assert self.fly_position is not None
        assert self.drone_position is not None

        relative_position = subtract(self.fly_position, self.drone_position)
        distance_m = norm(relative_position)
        self.current_distance_m = distance_m

        if distance_m < self.catch_distance_m:
            self.caught = True
            self.catch_time_s = now_s - self.start_time_s
            self.intercept_point = self.fly_position
            self._publish_stop()
            self.get_logger().info(
                "Fly caught! "
                f"distance={distance_m:.3f} m, "
                f"elapsed={self.catch_time_s:.2f} s"
            )
            return

        desired_speed = self._desired_speed(distance_m)
        intercept_time_s = self._calculate_intercept_time(
            relative_position,
            self.fly_velocity,
            max(desired_speed, 1e-3),
        )
        self.intercept_point = add_scaled(
            self.fly_position,
            self.fly_velocity,
            intercept_time_s,
        )

        target_vector = subtract(self.intercept_point, self.drone_position)
        horizontal_distance = math.hypot(target_vector[0], target_vector[1])
        yaw = math.atan2(target_vector[1], target_vector[0])
        pitch = math.atan2(target_vector[2], horizontal_distance)

        thrust = 100.0 * desired_speed / self.drone_speed_max_mps
        command = DroneCmd()
        command.thrust = float(clamp(thrust, 0.0, 100.0))
        command.pitch = float(pitch)
        command.yaw = float(yaw)
        self.command_publisher.publish(command)

    def _publish_stop(self) -> None:
        command = DroneCmd()
        command.thrust = 0.0
        command.pitch = 0.0
        command.yaw = 0.0
        self.command_publisher.publish(command)

    def _publish_status(self) -> None:
        """Publish concise progress information for tuning and review."""
        if self.caught:
            elapsed = 0.0 if self.catch_time_s is None else self.catch_time_s
            self.get_logger().info(
                f"Caught and holding position; elapsed={elapsed:.2f} s"
            )
            return
        if self.current_distance_m is None:
            self.get_logger().info("Waiting for valid measurements")
            return
        self.get_logger().info(
            f"distance={self.current_distance_m:.2f} m | "
            f"fly_speed_est={norm(self.fly_velocity):.2f} m/s | "
            f"drone_speed={norm(self.drone_velocity):.2f} m/s"
        )

    def _make_sphere_marker(
        self,
        marker_id: int,
        namespace: str,
        position: Vector3,
        scale: float,
        rgba: Tuple[float, float, float, float],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]
        marker.lifetime = Duration(seconds=0.3).to_msg()
        return marker

    def _publish_markers(self) -> None:
        """Visualize fly, drone, intercept point, and pursuit line in RViz2."""
        markers = MarkerArray()
        if self.fly_position is not None:
            markers.markers.append(
                self._make_sphere_marker(
                    0,
                    "fly",
                    self.fly_position,
                    0.25,
                    (1.0, 0.1, 0.1, 1.0),
                )
            )
        if self.drone_position is not None:
            markers.markers.append(
                self._make_sphere_marker(
                    1,
                    "drone",
                    self.drone_position,
                    0.40,
                    (0.1, 0.3, 1.0, 1.0),
                )
            )
        if self.intercept_point is not None:
            markers.markers.append(
                self._make_sphere_marker(
                    2,
                    "intercept",
                    self.intercept_point,
                    0.18,
                    (0.1, 1.0, 0.2, 0.9),
                )
            )
        if self.drone_position is not None and self.intercept_point is not None:
            line = Marker()
            line.header.frame_id = self.world_frame
            line.header.stamp = self.get_clock().now().to_msg()
            line.ns = "pursuit_line"
            line.id = 3
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.04
            line.color.r = 1.0
            line.color.g = 0.8
            line.color.b = 0.1
            line.color.a = 0.9
            line.lifetime = Duration(seconds=0.3).to_msg()

            start = PointStamped().point
            start.x, start.y, start.z = self.drone_position
            end = PointStamped().point
            end.x, end.y, end.z = self.intercept_point
            line.points = [start, end]
            markers.markers.append(line)

        self.marker_publisher.publish(markers)

    def stop(self) -> None:
        """Send several stop commands before shutdown."""
        for _ in range(3):
            self._publish_stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FlyCatcherController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
