from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the ros2socketcan_bridge for vcan0
        Node(
            package='ros2socketcan_bridge',
            executable='ros2socketcan',
            name='ros2socketcan_bridge_vcan0',
            parameters=[{'can_socket': 'vcan0'}],
        ),
        # Launch the ros2socketcan_bridge for vcan1
        # Node(
        #     package='ros2socketcan_bridge',
        #     executable='ros2socketcan',
        #     name='ros2socketcan_bridge_vcan1',
        #     parameters=[{'can_socket': 'vcan1'}],
        # ),
        # Launch relay node for vcan0
        Node(
            package='humble_mega_beetle',
            executable='cmd_vel_relay_node',
            name='relay_node_vcan0',
            parameters=[{'can_socket': 'vcan0'}],
        ),
        # Launch relay node for vcan1
        # Node(
        #     package='humble_mega_beetle',
        #     executable='relay_node',
        #     name='relay_node_vcan1',
        #     parameters=[{'can_socket': 'vcan1'}],
        # ),
    ])
