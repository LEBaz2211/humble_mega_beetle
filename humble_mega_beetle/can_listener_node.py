import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
from std_msgs.msg import Bool  # Assuming a simple Bool message to signal stop/start

class CANListener(Node):
    def __init__(self):
        super().__init__('can_listener')
        self.subscription = self.create_subscription(Frame, 'CAN/can0/receive', self.can_callback, 10)
        self.nav_control_publisher = self.create_publisher(Bool, 'navigation_control/stop', 10)

    def can_callback(self, msg):
        # Decode the CAN frame data
        # Assuming the first byte indicates the sensor type (e.g., 0x01 for proximity sensor)
        # and the second byte indicates the action (e.g., 0x00 for continue, 0x01 for stop)
        sensor_type, action = msg.data[0], msg.data[1]

        if sensor_type == 0x01:  # Proximity sensor
            if action == 0x01:  # Stop signal
                # self.nav_control_publisher.publish(Bool(data=True))
                self.get_logger().info('Proximity alert: Stopping navigation')
            elif action == 0x00:  # Continue signal
                # self.nav_control_publisher.publish(Bool(data=False))
                self.get_logger().info('Proximity clear: Resuming navigation')

def main(args=None):
    rclpy.init(args=args)
    node = CANListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
