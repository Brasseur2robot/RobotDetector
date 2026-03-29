#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from smbus2 import SMBus
from std_msgs.msg import UInt16MultiArray

DEBUG = True


class I2CSender(Node):
    def __init__(self):
        super().__init__("i2c_sender")

        # I2C Configuration
        self.i2c_bus = 1
        self.esp32_address = 0x08

        try:
            self.bus = SMBus(self.i2c_bus)
            self.get_logger().info(f"I2C initialized on bus {self.i2c_bus}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize I2C: {e}")
            return

        # Subscribe to the i2c_data
        self.subscription = self.create_subscription(
            UInt16MultiArray, "/i2c_data", self.data_callback, 10
        )

        self.get_logger().info("I2C Sender ready!")

    def data_callback(self, msg):

        # Extract values from array
        if len(msg.data) != 2:
            self.get_logger().warn(f"Expected 2 values, got {len(msg.data)}")
            return

        distance_mm = msg.data[0]
        angle_deg = msg.data[1]

        if DEBUG:
            self.get_logger().info(
                f"Theses data will be sent: distance={distance_mm}mm, angle={angle_deg}°"
            )

        try:
            self.i2c_sender(distance_mm, angle_deg)
        except Exception as e:
            self.get_logger().error(f"I2C transmission failed: {e}")

    def i2c_sender(self, distance, angle):
        """Send 2 uint16_t values (4 bytes) via I2C

        Bytes are send using Big-Endian so MSB comes first then LSB

        I2C transmission format:
            Byte 0: distance MSB
            Byte 1: distance LSB
            Byte 2: angle MSB
            Byte 3: angle LSB

        Args:
            distance (int): Distance in millimeter [0; 65 535]
            angle (int): Angle in degree [0; 65 535]
        """

        data = []

        for value in [distance, angle]:
            if value not in (0, 65535):
                self.get_logger().error("Value are not in a [0, 65535] range")

            # MSB and LSB conversion
            high_byte = (value >> 8) & 0xFF
            low_byte = value & 0xFF

            data.append(high_byte)
            data.append(low_byte)

            # Sending data over I2C
            self.bus.write_i2c_block_data(self.esp32_address, 0x00, data)

            if DEBUG:
                self.get_logger().debug(
                    f"I2C bytes sent: {[hex(b) for b in data]} "
                    f"(dist={distance}, angle={angle})"
                )

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
