from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    isuzu_map_params = {
        "latitude_map": 40.81187906,
        "longitude_map": 29.35810110,
        "altitude_map": 53.251157145314075

    }
    return LaunchDescription([
        Node(
            package='gnss_ins_msg_buffer',
            executable='gnss_ins_msg_buffer',
            name='gnss_ins_msg_buffer',
            parameters=[isuzu_map_params],
            output='screen'
        )

    ])