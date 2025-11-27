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

    MAX_DRONE_COUNT = 10  # REMEMBER TO CHANGE THIS IF YOU ADD MORE DRONES

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
        ),
        Node(
            package='droneswarm',
            namespace=namespace,
            executable='px4_controller',
            name='drone_4',
            parameters=[
                {'instance_id': 4}, 
                {'start_flight_delay_s': 15},
                {'max_drone_count': MAX_DRONE_COUNT},
                {'path_planning_method': 'partition_method'},
                {'use_sim_time': True}
            ]
        ),
        Node(
            package='droneswarm',
            namespace=namespace,
            executable='px4_controller',
            name='drone_5',
            parameters=[
                {'instance_id': 5}, 
                {'start_flight_delay_s': 20},
                {'max_drone_count': MAX_DRONE_COUNT},
                {'path_planning_method': 'partition_method'},
                {'use_sim_time': True}
            ]
        ),
        Node(
            package='droneswarm',
            namespace=namespace,
            executable='px4_controller',
            name='drone_6',
            parameters=[
                {'instance_id': 6}, 
                {'start_flight_delay_s': 25},
                {'max_drone_count': MAX_DRONE_COUNT},
                {'path_planning_method': 'partition_method'},
                {'use_sim_time': True}
            ]
        ),
        Node(
            package='droneswarm',
            namespace=namespace,
            executable='px4_controller',
            name='drone_7',
            parameters=[
                {'instance_id': 7}, 
                {'start_flight_delay_s': 30},
                {'max_drone_count': MAX_DRONE_COUNT},
                {'path_planning_method': 'partition_method'},
                {'use_sim_time': True}
            ]
        ),
        Node(
            package='droneswarm',
            namespace=namespace,
            executable='px4_controller',
            name='drone_8',
            parameters=[
                {'instance_id': 8}, 
                {'start_flight_delay_s': 35},
                {'max_drone_count': MAX_DRONE_COUNT},
                {'path_planning_method': 'partition_method'},
                {'use_sim_time': True}
            ]
        ),
        Node(
            package='droneswarm',
            namespace=namespace,
            executable='px4_controller',
            name='drone_9',
            parameters=[
                {'instance_id': 9}, 
                {'start_flight_delay_s': 40},
                {'max_drone_count': MAX_DRONE_COUNT},
                {'path_planning_method': 'partition_method'},
                {'use_sim_time': True}
            ]
        ),
        Node(
            package='droneswarm',
            namespace=namespace,
            executable='px4_controller',
            name='drone_10',
            parameters=[
                {'instance_id': 10}, 
                {'start_flight_delay_s': 45},
                {'max_drone_count': MAX_DRONE_COUNT},
                {'path_planning_method': 'partition_method'},
                {'use_sim_time': True}
            ]
        ),
    ])