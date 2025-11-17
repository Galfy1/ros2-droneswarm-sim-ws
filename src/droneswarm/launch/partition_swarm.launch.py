from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


# To avoid failsafe and refusal-to-arm issues when running the sim in WSL (due to WSL clock skew), we set each node to use Gazebo time instead of System time.
# (done by setting the 'use_sim_time' parameter to True)
# This also require you to install ros-humble-ros-gzharmonic and run a terminal with a specific "ros2 run ros_gz_bridge..." command to publish the /clock topic.
# For more info, see https://docs.px4.io/main/en/ros2/user_guide.html#ros-gazebo-and-px4-time-synchronization 


def generate_launch_description():

    # Declare the namespace argument (it can be provided when launching)
    namespace = LaunchConfiguration('namespace')

    MAX_DRONE_COUNT = 3  # REMEMBER TO CHANGE THIS IF YOU ADD MORE DRONES.
                         # IMPORTANT: THIS NUMBER MUST MATCH THE UAV COUNT USED IN THE OFFLINE PARTITIONING SCRIPT.

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='application',
            description='Namespace of the nodes'
        ),
        Node(
            package='droneswarm',
            namespace=namespace,
            executable='px4_controller',
            name='drone_1',
            parameters=[
                {'instance_id': 1}, # start from 1! not 0!
                {'start_flight_delay_s': 0}, # delay before starting the flight (seconds)
                {'max_drone_count': MAX_DRONE_COUNT},
                {'path_planning_method': 'partition_method'},
                {'use_sim_time': True}
            ]
        ),
        Node(
            package='droneswarm',
            namespace=namespace,
            executable='px4_controller',
            name='drone_2',
            parameters=[
                {'instance_id': 2},
                {'start_flight_delay_s': 5},
                {'max_drone_count': MAX_DRONE_COUNT},
                {'path_planning_method': 'partition_method'},
                {'use_sim_time': True}
            ]
        ),
        Node(
            package='droneswarm',
            namespace=namespace,
            executable='px4_controller',
            name='drone_3',
            parameters=[
                {'instance_id': 3}, 
                {'start_flight_delay_s': 10},
                {'max_drone_count': MAX_DRONE_COUNT},
                {'path_planning_method': 'partition_method'},
                {'use_sim_time': True}
            ]
        )
    ])