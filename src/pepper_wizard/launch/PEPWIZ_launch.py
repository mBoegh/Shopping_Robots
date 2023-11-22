from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('server_log_level', default_value='INFO'),
        DeclareLaunchArgument('gui_log_level', default_value='INFO'),

        Node(
            package='pepper_wizard',
            executable='server',
            name='server',
            parameters=[{'ros__parameters': {'ros__logging__logger_level': LaunchConfiguration('server_log_level')}}]
        ),
        
        Node(
            package='pepper_wizard',
            executable='gui',
            name='gui',
            parameters=[{'ros__parameters': {'ros__logging__logger_level': LaunchConfiguration('gui_log_level')}}]
        )
    ])
