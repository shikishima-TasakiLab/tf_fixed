from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf_fixed',
            executable='tf_fixed',
            name='tf_fixed',
            output='screen',
            parameters=[
                {'period_ms': 1000},

                {'frame_id': 'world'},
                {'child_frame_id': 'map'},

                {'translation.x': 1.0},
                {'translation.y': 2.0},
                {'translation.z': 3.0},

                {'use_quaternion': False},
                {'use_euler_rad': False},
                {'use_euler_degrees': True},

                {'rotation.quaternion.x': 0.0},
                {'rotation.quaternion.y': 0.0},
                {'rotation.quaternion.z': 0.0},
                {'rotation.quaternion.w': 1.0},

                {'rotation.euler_rad.roll': 0.0},
                {'rotation.euler_rad.pitch': 0.0},
                {'rotation.euler_rad.yaw': 0.0},

                {'rotation.euler_degrees.roll': 45.0},
                {'rotation.euler_degrees.pitch': 30.0},
                {'rotation.euler_degrees.yaw': 60.0},
            ]
        )
    ])