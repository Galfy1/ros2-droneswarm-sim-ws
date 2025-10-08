from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration



def generate_launch_description():

    # Declare the namespace argument (it can be provided when launching)
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='px4_offboard_ns',
            description='Namespace of the nodes'
        ),
        Node(
            package='droneswarm',
            namespace=namespace,
            executable='px4_controller',
            name='drone_1',
            parameters=[
                {'instance_id': 1} # start from 1! not 0!
            ]
        ),
        # Node(
        #     package='droneswarm',
        #     namespace=namespace,
        #     executable='px4_controller',
        #     name='drone_2',
        #     parameters=[
        #         {'instance_id': 2}
        #     ]
        # ),
        # Node(
        #     package='droneswarm',
        #     namespace=namespace,
        #     executable='px4_controller',
        #     name='drone_3',
        #     parameters=[
        #         {'instance_id': 3}
        #     ]
        # )
    ])