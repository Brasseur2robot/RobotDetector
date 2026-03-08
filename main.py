#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
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

        self.marker_pub = self.create_publisher(MarkerArray, "/detected_robots", 10)

        self.get_logger().info("Robot Detector started!")
        self.get_logger().info(
            f"Looking for robot {self.expected_width}m wide in front ±{self.front_angle}°"
        )

    def scan_callback(self, msg):
        # Get frame_id from laser scan for proper visualization
        self.frame_id = msg.header.frame_id

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

        # Create marker array for visualization
        marker_array = MarkerArray()
        marker_id = 0

        # Check each cluster
        for cluster in clusters:
            distance, width, angle = self.analyze_cluster(cluster)

            # Check if it matches our expected robot
            is_target = abs(width - self.expected_width) <= self.width_tol
            if is_target:
                self.get_logger().info(
                    f"🎯 ROBOT DETECTED! Distance: {distance:.2f}m, "
                    f"Width: {width:.2f}m, Angle: {angle:.1f}°"
                )
            # Create marker for this cluster
            marker = self.create_cluster_marker(cluster, marker_id, is_target)
            marker_array.markers.append(marker)
            marker_id += 1

            # Add text label
            text_marker = self.create_text_marker(
                cluster, marker_id, distance, width, is_target
            )
            marker_array.markers.append(text_marker)
            marker_id += 1

        # Publish markers
        self.marker_pub.publish(marker_array)

    def create_cluster_marker(self, cluster, marker_id, is_target):
        """Create a visual marker for a cluster"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "clusters"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Draw lines connecting all points in cluster
        for point in cluster:
            p = Point()
            p.x = point["x"]
            p.y = point["y"]
            p.z = 0.0
            marker.points.append(p)

        # Close the shape
        p = Point()
        p.x = cluster[0]["x"]
        p.y = cluster[0]["y"]
        p.z = 0.0
        marker.points.append(p)

        # Color: Green if target object, Yellow otherwise
        marker.scale.x = 0.02  # Line width
        if is_target:
            marker.color.r = 0.0
            marker.color.g = 1.0  # Green
            marker.color.b = 0.0
        else:
            marker.color.r = 1.0  # Yellow
            marker.color.g = 1.0
            marker.color.b = 0.0
        marker.color.a = 1.0

        return marker

    def create_text_marker(self, cluster, marker_id, distance, width, is_target):
        """Create text label showing distance and width"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Position text at center of cluster
        avg_x = sum(p["x"] for p in cluster) / len(cluster)
        avg_y = sum(p["y"] for p in cluster) / len(cluster)

        marker.pose.position.x = avg_x
        marker.pose.position.y = avg_y
        marker.pose.position.z = 0.2  # Slightly above

        marker.text = f"{distance:.2f}m\n{width:.2f}m wide"

        marker.scale.z = 0.1  # Text height

        # Color matches cluster
        if is_target:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        marker.color.a = 1.0

        return marker

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
