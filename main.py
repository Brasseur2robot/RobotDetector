#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import numpy as np


class RobotDetector(Node):
    def __init__(self):
        super().__init__("robot_detector")

        # Configuration
        self.declare_parameter("front_angle_range", 30.0)  # Check ±30° in front
        self.declare_parameter("min_distance", 0.1)  # Minimum valid distance (m)
        self.declare_parameter("max_distance", 5.0)  # Maximum detection range (m)
        self.declare_parameter(
            "cluster_tolerance", 0.1
        )  # Max gap between points in same robot (m)
        self.declare_parameter("expected_robot_width", 0.3)  # Expected robot width (m)
        self.declare_parameter("width_tolerance", 0.1)  # ±tolerance for width matching

        self.front_angle = self.get_parameter("front_angle_range").value
        self.min_dist = self.get_parameter("min_distance").value
        self.max_dist = self.get_parameter("max_distance").value
        self.cluster_tol = self.get_parameter("cluster_tolerance").value
        self.expected_width = self.get_parameter("expected_robot_width").value
        self.width_tol = self.get_parameter("width_tolerance").value

        # Subscribe to lidar
        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",  # Adjust topic name if needed
            self.scan_callback,
            10,
        )

        self.get_logger().info("Robot Detector started!")
        self.get_logger().info(
            f"Looking for robot {self.expected_width}m wide in front ±{self.front_angle}°"
        )

    def scan_callback(self, msg):
        # self.get_logger().info(
        #     f"📦 Received scan: {len(msg.ranges)} points, "
        #     f"angle range: {math.degrees(msg.angle_min):.1f}° to {math.degrees(msg.angle_max):.1f}°"
        # )

        # Print detailed info only for first scan
        if not hasattr(self, "_structure_printed"):
            self.get_logger().info("=== LaserScan Structure ===")
            self.get_logger().info(f"Frame: {msg.header.frame_id}")
            self.get_logger().info(f"Points: {len(msg.ranges)}")
            self.get_logger().info(
                f"Angle range: {math.degrees(msg.angle_min):.1f}° to {math.degrees(msg.angle_max):.1f}°"
            )
            self.get_logger().info(
                f"Angle step: {math.degrees(msg.angle_increment):.3f}°"
            )
            self.get_logger().info(
                f"Distance range: {msg.range_min}m to {msg.range_max}m"
            )
            self.get_logger().info(f"Sample ranges: {msg.ranges[:10]}")
            self.get_logger().info(f"Has intensities: {len(msg.intensities) > 0}")
            self._structure_printed = True

        # Extract front sector data
        front_points = self.get_front_sector(msg)
        print(front_points)

        # Full message
        # self.get_logger().info(f"Full message:\n{msg}")

        # Print front points in detail
        # for p in front_points[:5]:  # First 5 front points
        #     self.get_logger().info(
        #         f"  Point: x={p['x']:.2f}m, y={p['y']:.2f}m, "
        #         f"distance={p['distance']:.2f}m, angle={p['angle']:.1f}°"
        #     )

        if len(front_points) == 0:
            self.get_logger().info("Empty front sector data")
            return

        # Find clusters (potential robots)
        clusters = self.find_clusters(front_points)

        # Check each cluster
        for cluster in clusters:
            distance, width, angle = self.analyze_cluster(cluster)

            # Check if it matches our expected robot
            if abs(width - self.expected_width) <= self.width_tol:
                self.get_logger().info(
                    f"🎯 ROBOT DETECTED! Distance: {distance:.2f}m, "
                    f"Width: {width:.2f}m, Angle: {angle:.1f}°"
                )

    def get_front_sector(self, msg):
        """Extract points in front of robot (±front_angle degrees)"""
        points = []

        for i, distance in enumerate(msg.ranges):
            # Skip invalid readings
            if distance < msg.range_min or distance > msg.range_max:
                continue
            if distance < self.min_dist or distance > self.max_dist:
                continue

            # Calculate angle for this point
            angle = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle)

            # Normalize angle to -180 to 180
            while angle_deg > 180:
                angle_deg -= 360
            while angle_deg < -180:
                angle_deg += 360

            # Check if in front sector
            if abs(angle_deg) <= self.front_angle:
                # Convert to Cartesian coordinates
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                points.append(
                    {"x": x, "y": y, "distance": distance, "angle": angle_deg}
                )

        return points

    def find_clusters(self, points):
        """Group nearby points into clusters (robots)"""
        if len(points) == 0:
            return []

        # Sort by angle for easier clustering
        points = sorted(points, key=lambda p: p["angle"])

        clusters = []
        current_cluster = [points[0]]

        for i in range(1, len(points)):
            prev_point = points[i - 1]
            curr_point = points[i]

            # Calculate distance between consecutive points
            dx = curr_point["x"] - prev_point["x"]
            dy = curr_point["y"] - prev_point["y"]
            gap = math.sqrt(dx * dx + dy * dy)

            if gap <= self.cluster_tol:
                # Same robot
                current_cluster.append(curr_point)
            else:
                # New robot - save previous cluster
                if len(current_cluster) >= 3:  # Need at least 3 points for valid robot
                    clusters.append(current_cluster)
                current_cluster = [curr_point]

        # Don't forget last cluster
        if len(current_cluster) >= 3:
            clusters.append(current_cluster)

        return clusters

    def analyze_cluster(self, cluster):
        """Calculate distance, width, and angle of a cluster"""
        # Average distance to robot
        avg_distance = sum(p["distance"] for p in cluster) / len(cluster)

        # Calculate width (distance between leftmost and rightmost points)
        leftmost = min(cluster, key=lambda p: p["y"])
        rightmost = max(cluster, key=lambda p: p["y"])

        dx = rightmost["x"] - leftmost["x"]
        dy = rightmost["y"] - leftmost["y"]
        width = math.sqrt(dx * dx + dy * dy)

        # Average angle
        avg_angle = sum(p["angle"] for p in cluster) / len(cluster)

        return avg_distance, width, avg_angle


def main(args=None):
    rclpy.init(args=args)
    detector = RobotDetector()

    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
