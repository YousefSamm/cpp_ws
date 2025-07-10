from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_pkg',
            executable='rpm_pub',
            name='rpm_node',
            output='screen',
            parameters=[{
                'rpm_value': 20.0
            }]
        ),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/speed'],
            output='screen'
        ),
        Node(
            package='cpp_pkg',
            executable='speed_calc',
            name='speed_subscriber',    
            output='screen',
            parameters=[{
                'wheel_radius': 0.3
            }]
        ),
        Node(
            package='cpp_pkg',
            executable='speed_sub.py',
            output='screen'
        )

    ])