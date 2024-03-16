import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from can_msgs.msg import Frame
import numpy as np
import struct

class TopicRelay(Node):
    def __init__(self, can_socket):
        super().__init__('topic_relay')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
        self.publisher = self.create_publisher(Frame, f'CAN/{can_socket}/transmit', 10)

    def listener_callback(self, msg):
        frame = Frame()
        frame.is_extended = False
        frame.id = 0x123
        frame.dlc = 8

        # Scale and convert the floating-point numbers to int16
        linear_x_int = int(msg.linear.x * 100)
        linear_y_int = int(msg.linear.y * 100)
        angular_z_int = int(msg.angular.z * 100)

        # Pack these integers into bytes using 'h' format for int16
        packed_data = struct.pack('hhh', linear_x_int, linear_y_int, angular_z_int)

        # Check if packed data exceeds 6 bytes (leaving 2 bytes for padding)
        if len(packed_data) > 6:
            self.get_logger().error('Packed data exceeds 6 bytes, unable to fit into CAN frame with padding.')
            return
        else:
            # Ensure the packed data fits the CAN frame data size (8 bytes)
            # Here, we fill the remaining 2 bytes with zeros
            frame.data = np.frombuffer(packed_data + b'\x00\x00', dtype=np.uint8)

        # Decode for logging (demonstration purposes)
        decoded_linear_x_int, decoded_linear_y_int, decoded_angular_z_int = struct.unpack('hhh', packed_data)
        self.get_logger().info(
            f'Decoded Velocities: Linear X: {decoded_linear_x_int / 100.0}, '
            f'Linear Y: {decoded_linear_y_int / 100.0}, Angular Z: {decoded_angular_z_int / 100.0}'
        )

        self.publisher.publish(frame)
        self.get_logger().info('Published CAN Frame with encoded Twist message')



def main(args=None):
    rclpy.init(args=args)
    can_socket = 'vcan0'  # Default value
    relay_node = TopicRelay(can_socket)

    try:
        rclpy.spin(relay_node)
    except KeyboardInterrupt:
        relay_node.get_logger().info('Node interrupted by keyboard, shutting down...')
    finally:
        relay_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
