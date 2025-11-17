THIS CODE IS NOT COMPLETE

# ros2-droneswarm-sim-ws
ROS2 workspace made for a drone swarm simulation using PX4/ROS2 Multi-Vehicle Simulation in Gazebo.  

# Get Started
1. Clone repo:
    - `git clone --recursive {this repo}`
    - `cd ros2-droneswarm-sim-ws`
1. Install PX4, ROS2 and setup Micro XRCE-DDS Agent:
    - Go to: https://docs.px4.io/main/en/ros2/user_guide#installation-setup and complete the following sections:
      - **"Install PX4"**
      - **"Install ROS 2"**
      - "**Setup Micro XRCE-DDS Agent & Client"**
        - (here, you only need step 1 and 2!)
1. Install QCC (required for simulation):
    -  https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html 
1. Install dependencies (stand in /ros2-droneswarm-sim-ws):
    - `sudo apt update`
    - `sudo rosdep init`
    - `rosdep update`
    - `source /opt/ros/humble/setup.bash`
    - `rosdep install -i --from-path src --rosdistro humble -y`
    - `sudo apt install python3-pip`
    - `python3 -m pip install -r requirements.txt`
    - `sudo apt install ros-humble-ros-gzharmonic`
1. Build ROS2 workspace (stand in /ros2-droneswarm-sim-ws):
    - `source /opt/ros/humble/setup.bash` 
    - `colcon build`
  
# How to Run / Launch
1. Open a new terminal (never run/launch in the same terminal you build the workspace)
1. Source overlay:
    - `cd {path to the workspace root}`
    - `source install/setup.bash`
1. Run or Launch whatever you want. e.g.:
    - `ros2 launch {package name} {launch file name}`
  



# About Dependencies

To add a dependency:
- add it to package.xml within the ROS2 package

If you need a Python package that is NOT part of rosdep 
- (see https://docs.ros.org/en/humble/How-To-Guides/Using-Python-Packages.html for lists of available Python packages in rosdep):
- add it to requirements.txt

For a list of general ROS2 packages, see:
- [ROS Index](https://index.ros.org/?search_packages=true)
- (you can also depend on these in the package.xml file)

# About Time
## About WSL Clock Skew
### PROBLEM
WSL clock skew can sometimes lead to very weird behaviour in the sim. These issues are very hard to debug!  
Symptoms include:
- Drones randomly refusing to arm
- Constant failsafes due to “No offboard signal”
if clock skew exceeds COM_OF_LOSS_T → failsafe.  
(due to wrong timestamps published to the /fmu/in/offboard_control_mode topic)

### SOLUTION  
Setup nodes to use Gazebo clock instead of OS clock (requires ros_gz_bridge package). In this repo, everything that needs to be set up in the source code to achieve this has already been done (in the Launch file) - however, you still need a terminal to run ros_gz_bridge (see "A typical Work Setup" below).  
For more info: https://docs.px4.io/main/en/ros2/user_guide.html#ros-gazebo-and-px4-time-synchronization

## Increase Sim Speed
To increase sim speed, add the PX4_SIM_SPEED_FACTOR=<speed_factor> flag to the command used when spawning the dds clients (see "A Typical Work Setup" below). Set the speed_factor to whatever you want, however your computer might not be able to run it if set to high. From PX4 doc: "Powerful desktop machines can usually run the simulation at around 6-10x, for notebooks the achieved rates can be around 3-4x". However, increasing the speed seems to slightly alter the behavior of the drones. For the highest accuracy, 1× speed seems to be best.

_**NOTE:** Increasing this value too high for your system might lead to a broken PX4-Autopilot installation.
Symptoms: Drone refusing to arm, with QGC errors like "Navigation error: No valid position estimate" or "Navigation error: No valid global position estimate".
To fix this, delete your PX4-Autopilot folder and reinstall it according to the "Install PX4" section in this guide: [PX4 ROS2 User Guide](https://docs.px4.io/main/en/ros2/user_guide.html#install-px4)._

# A Typical Work Setup

- **Terminal 1** (connect QGC):
    - STAND WHERE YOUR .AppImage IS LOCATED
    - `./QGroundControl-x86_64.AppImage`
- **Terminal 2** (multiple drones):
    - (multiple individual terminals)
    - Terminal 2a (gazebo simulation + dds client)
        - `cd ~/PX4-Autopilot`
        - `PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD=baylands PX4_SIM_SPEED_FACTOR=1 ./build/px4_sitl_default/bin/px4 -i 1`
    - (be sure the gazebo simulation has completely launched before spawning additional drones)
    - Terminal 2b (dds client)
        - `cd ~/PX4-Autopilot`
        - `PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,1" PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD=baylands  PX4_SIM_SPEED_FACTOR=1 ./build/px4_sitl_default/bin/px4 -i 2`
    - Terminal 2c (dds client)
      - `cd ~/PX4-Autopilot`
      - `PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,2" PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD=baylands  PX4_SIM_SPEED_FACTOR=1 ./build/px4_sitl_default/bin/px4 -i 3`
  - …
- **Terminal 3** (start agent + dds server):
    - `MicroXRCEAgent udp4 -p 8888`
- **Terminal 4** (run ros_gz_bridge for using Gazebo as time source for nodes):
    - `source /opt/ros/humble/setup.bash`
    - `ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock`
- **Terminal 5** (to run/launch stuff):
    - STAND IN YOUR ROS2 WORKSPACE
    - `source install/setup.bash`
    - RUN/LAUNCH WHATEVER YOU WANT
- **Terminal 6** (Optional) (to build ROS2 workspace):
    - STAND IN YOUR ROS2 WORKSPACE
    - `source /opt/ros/humble/setup.bash`
    - `colcon build`
- **Terminal 7** (Optional) (to view streamed drone camera footage)
    - YOU NEED A DRONE MODEL WITH CAMERA
    - (e.g. gz_x500_mono_cam_down)
    - (you can specify model when you start the dds client)
    - ```
      gst-launch-1.0 udpsrc port=5600 \
      ! application/x-rtp,encoding-name=H264,payload=96 \
      ! rtph264depay \
      ! avdec_h264 \
      ! videoconvert \
      ! autovideosink
      ```

# Miscellaneous Notes
If you keep seeing this warning: `WARN  [commander] Arming denied: Resolve system health failures first` try to delete the PX4-Autopilot folder and redo the Install PX4 step.



