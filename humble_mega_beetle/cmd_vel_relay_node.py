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
        packed_data = struct.pack('ff', msg.linear.x, msg.angular.z)
        frame.data = np.frombuffer(packed_data, dtype=np.uint8)
        self.publisher.publish(frame)
        self.get_logger().info('Published CAN Frame with encoded Twist message')

def main(args=None):
    rclpy.init(args=args)
    can_socket = 'vcan0'  # Default value, consider retrieving this from command line arguments or parameters
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
