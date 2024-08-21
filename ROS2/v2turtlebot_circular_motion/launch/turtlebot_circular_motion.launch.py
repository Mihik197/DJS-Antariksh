from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v2turtlebot_circular_motion',
            executable='radius_publisher',
            name='radius_publisher'
        ),
        Node(
            package='v2turtlebot_circular_motion',
            executable='angular_velocity_service',
            name='angular_velocity_service'
        ),
        Node(
            package='v2turtlebot_circular_motion',
            executable='turtlebot_controller',
            name='turtlebot_controller'
        ),
        Node(
            package='v2turtlebot_circular_motion',
            executable='figure_eight_publisher',
            name='figure_eight_publisher'
        )
    ])