THIS CODE IS NOT COMPLETE

# ros2-droneswarm-sim-ws
ROS2 workspace made for PX4 Multi-Vehicle Simulation in Gazebo.


# Get started
1. Clone repo:
    - git clone {this repo}
    - cd ros2-droneswarm-sim-ws
1. Install dependencies (stand in /ros2-droneswarm-sim-ws):
    - sudo apt update
    - rosdep update
    - source /opt/ros/humble/setup.bash
    - rosdep install -i --from-path src --rosdistro humble -y
1. Build ROS2 workspace (stand in /ros2-droneswarm-sim-ws):
    - source /opt/ros/humble/setup.bash 
    - colcon build --packages-up-to droneswarm

# How to Run / Launch
1. Open a new terminal (never run/launch in the same terminal you build the workspace)
1. Source overlay:
    - cd {path to the workspace root}
    - source install/setup.bash
1. Run or Launch whatever you want. e.g.:
    - ros2 launch {package name} {launch file name}  
