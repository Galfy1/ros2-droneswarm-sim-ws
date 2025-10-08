import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleGlobalPosition, HomePosition
import os
import pickle


from .map_projection import MapProjectionImpl
from .tsunami_online import tsunami_online_init, tsunami_online_loop 

# TODO behold de relevant nedersteående links
# see https://docs.px4.io/main/en/ros2/px4_ros2_control_interface.html for topic interface descriptions
# se https://docs.px4.io/v1.12/en/simulation/gazebo.html#set-world-location 
# se https://docs.px4.io/main/en/sim_gazebo_gz/#set-custom-takeoff-location 
# det ser ud til gps koordinater er defineret i maps filen: https://github.com/PX4/PX4-gazebo-models/blob/main/worlds/baylands.sdf (i bunden af filen)
    # det kan jeg eventuel bruge til at lave et polygon i mission planner!

# see https://github.dev/PX4/px4_msgs for message definitions
#     # https://docs.px4.io/main/en/msg_docs/  
# TrajectorySetpoint is for setpoints in meters
# PositionSetpoint seems to be setpoint for gps coordinates ????

# ITS BETTER TO JUST LOOK IN THE ACTUEL px4_msgs PACKAGE IN THE WORKSPACE, THEN YOU KNOW ITS THE ONE ACTUALLY BEING USED

package_name = 'droneswarm'


CONTROL_LOOP_DT = 0.05  # seconds
HOME_POS_TOLERANCE = 1e-4  # degrees # TODO ADJUST


class PX4_Controller(Node):

    def __init__(self) -> None:
        super().__init__('px4_controller')

        self.declare_parameter('instance_id', 0)
        self.instance_id = self.get_parameter('instance_id').get_parameter_value().integer_value
        self.ns = "/px4_" + str(self.instance_id) # Namespace for multiple vehicle instances

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, self.ns + '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, self.ns + '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, self.ns + '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, self.ns + '/fmu/out/vehicle_local_position_v1', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, self.ns + '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile)
        self.vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition, self.ns + '/fmu/out/vehicle_global_position', self.vehicle_global_position_callback, qos_profile)
        self.home_position_subscriber = self.create_subscription(
            HomePosition, self.ns + '/fmu/out/home_position_v1', self.home_position_callback, qos_profile)
            # NOTE: /px4_1/fmu/out/vehicle_gps_position also exist. for more info: https://docs.px4.io/main/en/msg_docs/ 

        # Read traversal order and home position from file (created in offline phase)
        pkl_path = os.path.join(os.path.join(os.path.dirname(__file__),'../','../','../','../','share', package_name, 'our_data'), 'traversal_order_gps.pkl')
        with open(pkl_path, 'rb') as fp:
            data_loaded = pickle.load(fp) 
        # for item in os.listdir(dir_path):
        #     self.get_logger().info(f"Found item in parent dir: {item}")
        self.home_pos_gps_from_offline = data_loaded['home_pos_gps']
        self.traversal_order_gps = data_loaded['traversal_order_gps']
        
        # Initialize  variables
        self.one_sec_loop_count = int(1.0 / CONTROL_LOOP_DT)
        self.offboard_startup_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.vehicle_global_position = VehicleGlobalPosition()
        self.home_pos = HomePosition()
        self.map_projection_initialized = False

        # Create a timer to publish control commands
        self.timer = self.create_timer(CONTROL_LOOP_DT, self.controll_loop_callback)

        # Init Tsunami
        tsunami_online_init(self)


    ##################### METHODS #####################

    def vehicle_local_position_callback(self, vehicle_local_position):
        if vehicle_local_position.xy_valid and vehicle_local_position.z_valid:
            #self.get_logger().info(f"Local Position: {vehicle_local_position.x}, {vehicle_local_position.y}, {vehicle_local_position.z}")
            self.vehicle_local_position = vehicle_local_position

    def home_position_callback(self, home_position): # https://docs.px4.io/main/en/msg_docs/HomePosition.html 
        # Initialize map projection using home position from PX4.
        if home_position.valid_hpos and not self.map_projection_initialized:
            # Check if home position matches the one from offline phase (within a small tolerance)
            if not (abs(home_position.lat - self.home_pos_gps_from_offline[0]) < HOME_POS_TOLERANCE and abs(home_position.lon - self.home_pos_gps_from_offline[1]) < HOME_POS_TOLERANCE):
                raise ValueError(f"Home position from PX4 does not match the one from offline phase (within a tolerance of {HOME_POS_TOLERANCE} degrees).")
            self.home_pos = home_position
            self.map_projection = MapProjectionImpl(self.home_pos.lat, self.home_pos.lon) # Initialize map projection with actual home position from PX4
            self.map_projection_initialized = True
            

    def vehicle_global_position_callback(self, vehicle_global_position): # https://docs.px4.io/main/en/msg_docs/VehicleGlobalPosition.html
        # Update vehicle global position
        if vehicle_global_position.lat_lon_valid and vehicle_global_position.alt_valid:
            self.vehicle_global_position = vehicle_global_position

    def vehicle_status_callback(self, vehicle_status):
        # Update vehicle status
        self.vehicle_status = vehicle_status

    def arm(self):
        # Send an arm command to the vehicle.
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        # Send a disarm command to the vehicle.
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        # Switch to offboard mode.
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        # Switch to land mode.
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Landing vehicle")

    def publish_offboard_control_heartbeat_signal(self):
        # Publish the offboard control mode.
        # see: https://docs.px4.io/main/en/flight_modes/offboard.html 
        msg = OffboardControlMode()
        msg.position = True 
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint_local(self, x: float, y: float, z: float, velocity: float = 1.0, yaw: float = 0.0):
        # Publish the trajectory setpoint.
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        #msg.velocity = [velocity, velocity, velocity] #DOES NOT WORK FOR WHATEVER REASON
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        #self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_position_setpoint_global(self, lat: float, lon: float, alt: float, velocity: float = 1.0, yaw: float = 0.0):
        x, y = self.map_projection.global_to_local(lat, lon) # px does not take global setpoints in python. so we convert to local
        self.publish_position_setpoint_local(x, y, alt, velocity, yaw=yaw)

    def publish_vehicle_command(self, command, **params) -> None:
        # Publish a vehicle command.
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = self.instance_id + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)


    ##################### MAIN CONTROL LOOP #####################


    def controll_loop_callback(self) -> None:

        self.publish_offboard_control_heartbeat_signal() # must be called at least at 2Hz

        # Wait a second before starting offboard mode and arming (required by PX4)
        if self.offboard_startup_counter == self.one_sec_loop_count:
            self.engage_offboard_mode()
            self.arm()
            self.offboard_startup_counter = 9999  # Prevent re-entering this if statement
        if self.offboard_startup_counter < self.one_sec_loop_count:
            self.offboard_startup_counter += 1

        # If PX4 is in offboard mode and map projection is initialized, run the main control loop
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.map_projection_initialized:
            tsunami_online_loop(self)
            




        





def main(args=None) -> None:
    print('Starting PX4 Controller...')
    rclpy.init(args=args)

    px4_controller = PX4_Controller()
    rclpy.spin(px4_controller)

    px4_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)