#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from smbus2 import SMBus
from std_msgs.msg import Float32MultiArray


class I2CSender(Node):
    def __init__(self):
        super().__init__("i2c_sender")

        # I2C Configuration
        self.i2c_bus = 1
        self.esp32_address = 0x08  # Need to be checked

        try:
            self.bus = SMBus(self.i2c_bus)
            self.get_logger().info(f"I2C initialized on bus {self.i2c_bus}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize I2C: {e}")
            return

        # Subscribe to the i2c_data
        self.subscription = self.create_subscription(
            Float32MultiArray, "/i2c_data", self.data_callback, 10
        )

        self.get_logger().info("I2C Sender ready!")

    def data_callback(self, msg):

        # Extract values from array
        if len(msg.data) != 2:
            self.get_logger().warn(f"Expected 2 values, got {len(msg.data)}")
            return

        distance = msg.data[0]
        angle = msg.data[1]

        self.get_logger().info(
            f"Theses data will be sent: distance={distance}m, angle={angle}°"
        )

    def i2c_sender(self, distance, angle):

        data = []

        # for value in [distance, angle]:

    def __del__(self):
        """Clean up I2C bus when node is destroyed"""

        if hasattr(self, "bus"):
            self.bus.close()


def main(args=None):
    rclpy.init(args=args)
    sender = I2CSender()

    try:
        rclpy.spin(sender)
    except KeyboardInterrupt:
        pass
    finally:
        sender.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
