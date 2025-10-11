import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleGlobalPosition, HomePosition
from std_msgs.msg import Int32
import os
import pickle

from .map_projection import MapProjectionImpl
from .tsunami_online import tsunami_online_init, tsunami_online_loop 
from our_custom_interfaces.srv import SyncVisitedWaypoints

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

# For Interface Definitions, see: https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html and https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html 

# For thread control info, see https://docs.ros.org/en/humble/How-To-Guides/Sync-Vs-Async.html (tldr: dont use synchronous clients in callbacks, deadlock will occur! use call_async() instead)
#                              https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html (really good read!) 

package_name = 'droneswarm'


CONTROL_LOOP_DT = 0.05  # seconds
HOME_POS_TOLERANCE = 1e-4  # degrees # TODO ADJUST
REQUEST_ALL_CURRENT_PATHS_TIMEOUT = 0.1 # seconds # TODO ADJUST
                                        # TODO, vi kan init med max antal drones. hvis den får responses for det antal, så er den færdig.
                                                # hvis en eller flere droner ikke svarer, så er det timeouten der bestemmer hvornår den er færdig
                                                #   hvis man ikke får svar fra en drone x antal gange i træk, så decrase antallen af droner i poolen med 1
                                                                    # kald variablen noget alla drone_pool_count_estimate eller lignende
                                                #   hvis man får svar fra en drone, så reset tælleren for den drone.
                                                #   PROBLEM: hvis vi har decreased antallet af droner i pooolen, så kan en drone der kommer online igen have svært ved at komme med igen (fordi vi kun venter på drone_pool_count_estimate svar)
                                                #      Potential løsning: efter man har fået drone_pool_count_estimate svar, så venter man lige lidt ekstra, så hvis en drone kommer online igen, så kan den nå at være med.
                                                # Det kræver også hver drone har et unikt ID, så vi kan se hvem svarer
SYNC_VISITED_WAYPOINTS_TIMEOUT = 0.1 # seconds. timeout for the client request to get the visited_waypoints lists from all other drones


class PX4_Controller(Node):

    def __init__(self) -> None:
        super().__init__('px4_controller')

        self.declare_parameter('instance_id', 1) # start ID's from 1 (not 0!) 
        self.declare_parameter('max_drone_count', 1) # its asumed that the instance_id is in the range [1, max_drone_count]
        self.declare_parameter('start_flight_delay_s', 0) # delay before starting the flight (seconds)

        self.instance_id = self.get_parameter('instance_id').get_parameter_value().integer_value
        self.max_drone_count = self.get_parameter('max_drone_count').get_parameter_value().integer_value
        self.start_flight_delay_s = self.get_parameter('start_flight_delay_s').get_parameter_value().integer_value

        # Namespaces
        self.ns_px4 = "/px4_" + str(self.instance_id) # Namespace for multiple vehicle instances. 
        self.ns_drone = "drone_" + str(self.instance_id) # Namespace we use for communication between drones
        self.ns_launch = self.get_namespace() # Namespace from the launch file


        # Error check for instance_id
        if not (1 <= self.instance_id <= self.max_drone_count):
            raise ValueError(f"instance_id must be in the range [1, {self.max_drone_count}]") # px4 instance_id starts from 1 when using multiple vehicle simulation, and this files asumes this.

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, self.ns_px4 + '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, self.ns_px4 + '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, self.ns_px4 + '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, self.ns_px4 + '/fmu/out/vehicle_local_position_v1', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, self.ns_px4 + '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile)
        self.vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition, self.ns_px4 + '/fmu/out/vehicle_global_position', self.vehicle_global_position_callback, qos_profile)
        self.home_position_subscriber = self.create_subscription(
            HomePosition, self.ns_px4 + '/fmu/out/home_position_v1', self.home_position_callback, qos_profile)
            # NOTE: /px4_1/fmu/out/vehicle_gps_position also exist. for more info: https://docs.px4.io/main/en/msg_docs/ 


        ############ COMMUNICATION STUFF ############

        # Create publishers
        self.visited_waypoint_publisher = self.create_publisher(
            Int32, '/visited_waypoint', qos_profile) # global topic, all drones publish and subscribe to the same topic (i.e. no individual drone namespace)

        # Create subscribers
        self.visited_waypoint_subscriber = self.create_subscription(
            Int32, '/visited_waypoint', self.visited_waypoint_callback, qos_profile)

        # Create services
        self.sync_visited_waypoints_service = self.create_service(SyncVisitedWaypoints, self.ns_drone + '/sync_visited_waypoints', self.sync_visited_waypoints_server_callback)

        # Create clients
        # (for each service, we need a client for each drone)
        self.sync_visited_waypoints_clients = [] 
        for drone_id in range(1, self.max_drone_count + 1): # start from 1 to max_drone_count (inclusive)
            if drone_id == self.instance_id:
                continue # skip self
            drone_ns = "drone_" + str(drone_id)
            self.sync_visited_waypoints_clients.append(self.create_client(SyncVisitedWaypoints, self.ns_launch +"/"+ drone_ns + '/sync_visited_waypoints'))
            while not self.sync_visited_waypoints_clients[-1].wait_for_service(timeout_sec=1.0): # (-1 gets the last added client)
                self.get_logger().info('service not available, waiting again...')




        # Initialize  variables for communication
    


        ############ COMMUNICATION STUFF END ############ 
         
        

        # Read traversal order and home position from file (created in offline phase)
        pkl_path = os.path.join(os.path.join(os.path.dirname(__file__),'../','../','../','../','share', package_name, 'our_data'), 'traversal_order_gps.pkl')
        with open(pkl_path, 'rb') as fp:
            data_loaded = pickle.load(fp) 
        # for item in os.listdir(dir_path):
        #     self.get_logger().info(f"Found item in parent dir: {item}")
        self.home_pos_gps_from_offline = data_loaded['home_pos_gps']
        self.traversal_order_gps = data_loaded['traversal_order_gps']
        self.traversal_order_size = len(self.traversal_order_gps)

        #self.visited_waypoints = [False] * len(self.traversal_order_gps) # keep track of which waypoints have been visited
        
        # Initialize  variables
        self.one_sec_loop_count = int(1.0 / CONTROL_LOOP_DT)
        self.start_flight_delay_loops = int(float(self.start_flight_delay_s) / CONTROL_LOOP_DT)
        self.offboard_startup_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.vehicle_global_position = VehicleGlobalPosition()
        self.home_pos = HomePosition()
        self.map_projection_initialized = False

        # Create a timer to publish control commands
        self.timer = self.create_timer(CONTROL_LOOP_DT, self.control_loop_callback)

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


    ##################### METHODS FOR COMMUNICATION #####################

    # def request_all_current_paths(self): # TODO service
    #     pass

    # def receive_all_current_paths_callback(self): 
    #     pass



    # def current_path_response(self): # TODO service
    #     pass

    # def broadcast_visited_waypoint(self): # broadcast = publish topic. its called broadcast for generilization with the tsunami implimentation for real hardware 
    #     pass

    # def visited_waypoint_callback(self): # TODO topic
    #     pass





    def broadcast_visited_waypoint(self, waypoint_index): # Publish Topic (its called broadcast for generilization with the tsunami implimentation for real hardware)
        # Broadcast that we have visited a waypoint by publishing to a topic
        if 0 <= waypoint_index < len(self.visited_waypoints):
            self.get_logger().info(f"Broadcasting visited waypoint {waypoint_index}")
            msg = Int32()
            msg.data = waypoint_index
            self.visited_waypoint_publisher.publish(msg)
        else:
            self.get_logger().warn(f"Invalid waypoint index {waypoint_index} for broadcasting visited waypoint")

    def visited_waypoint_callback(self, msg): # Topic subscriber callback
        # This is called when we receive a broadcast from another drone that it has visited a waypoint
        self.get_logger().info(f"Received visited waypoint {msg.data}. Marking as visited locally.")
        waypoint_index = msg.data
        if 0 <= waypoint_index < len(self.visited_waypoints):
            self.visited_waypoints[waypoint_index] = True # mark waypoint as visited locally
        else:
            self.get_logger().warn(f"Invalid waypoint index {waypoint_index} received in visited waypoint callback")



    def sync_visited_waypoints(self): # service client
        # Request the "visited_waypoints" list from all other drones, and update our own to the union of all lists.
        # Calling this method at the start, will allow for late-joining drones to get the current state

        self.get_logger().info("Syncing visited waypoints with all other drones...")

        for client in self.sync_visited_waypoints_clients:
            req = SyncVisitedWaypoints.Request()

            future = client.call_async(req)
            # Note: dont use .spin_until_future_complete here! it will deadlock! Because sync_visited_waypoints() is being run from the main controll loop (i.e. a timer callback!)
            # PROBLEM: Yes we use call_async(), but .spin_until_future_complete is blocking its asociated callback group! - and callbacks are needed to process the response. BUT callbacks cant be called because we are already in a timer callback and all the callbacks are in the same callback group!
            #          For more info: see https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html (they explain the problem very good)
            # SOLUTION 1: Use MultiThreadedExecutor and assign the callbacks different callback groups (by default, they all use the same). Then its allowed to block the callback!
            # SOLUTION 2 (our choice): Use call_async() and setup a callback for when the future is done (i.e. completety non-blocking and asyncronoues)
            # rclpy.spin_until_future_complete(self, future,) #timeout_sec=SYNC_VISITED_WAYPOINTS_TIMEOUT) # This is blocking
            future.add_done_callback(self.sync_visited_waypoints_client_future_callback)

    def sync_visited_waypoints_client_future_callback(self, future):
        if future.result() is not None:
            response = future.result()
            #self.get_logger().info(f"Received visited waypoints: {response.visited_waypoints}")
            # Update our own visited_waypoints to the union of both lists
            self.visited_waypoints = [a or b for a, b in zip(self.visited_waypoints, response.visited_waypoints)]
        else:
            self.get_logger().warn('Failed to receive visited waypoints. (Future is None)')

    def sync_visited_waypoints_server_callback(self, request, response): # service server callback
        # this is called when another drone requests our visited_waypoints list
        response.visited_waypoints = self.visited_waypoints
        #self.get_logger().info(f"Sending visited waypoints to requester: {response.visited_waypoints}")
        return response

    # def current_path_incoming_request_callback(self): # DET SKAL VÆRE EN SERVICE I STEDET TODO
    #     pass



    ##################### MAIN CONTROL LOOP #####################


    def control_loop_callback(self) -> None:
        start_time = self.get_clock().now()

        self.publish_offboard_control_heartbeat_signal() # must be called at least at 2Hz

        #self.get_logger().info(f"SUSHIIII")

        # Wait atleast a second before starting offboard mode and arming (required by PX4)
        wait_loops = self.one_sec_loop_count + self.start_flight_delay_loops
        if self.offboard_startup_counter == wait_loops:
            self.engage_offboard_mode()
            self.arm()
            self.sync_visited_waypoints() # Sync visited waypoints with all other drones
            self.offboard_startup_counter = 9999  # Prevent re-entering this if statement
        if self.offboard_startup_counter < wait_loops:
            self.offboard_startup_counter += 1

        # If PX4 is in offboard mode and map projection is initialized, run the main control loop
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.map_projection_initialized:   
            tsunami_online_loop(self)

        # ERROR CHECKING - Make sure control loop processing time does not exceed CONTROL_LOOP_DT:
        end_time = self.get_clock().now()
        elapsed_time_s = (end_time - start_time).nanoseconds / 1e9
        if elapsed_time_s > CONTROL_LOOP_DT:
            self.get_logger().warn(f"Control loop overrun: {elapsed_time_s:.4f} seconds")
            




        





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