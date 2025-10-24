import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleGlobalPosition, HomePosition
from std_msgs.msg import Int32
import os
import pickle

from .map_projection import MapProjectionImpl
from .tsunami_online import tsunami_online_init, tsunami_online_loop, tsunami_does_paths_cross
from our_custom_interfaces.srv import SyncVisitedCells, CheckAllCurrentPaths # custom ROS2 interfaces
from our_custom_interfaces.msg import Cell, Path # custom ROS2 interfaces

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
CHECK_ALL_CURRENT_PATHS_TIMEOUT = 2.0 # seconds # TODO ADJUST
                                        # TODO, vi kan init med max antal drones. hvis den får responses for det antal, så er den færdig.
                                                # hvis en eller flere droner ikke svarer, så er det timeouten der bestemmer hvornår den er færdig
                                                #   hvis man ikke får svar fra en drone x antal gange i træk, så decrase antallen af droner i poolen med 1
                                                                    # kald variablen noget alla drone_pool_count_estimate eller lignende
                                                #   hvis man får svar fra en drone, så reset tælleren for den drone.
                                                #   PROBLEM: hvis vi har decreased antallet af droner i pooolen, så kan en drone der kommer online igen have svært ved at komme med igen (fordi vi kun venter på drone_pool_count_estimate svar)
                                                #      Potential løsning: efter man har fået drone_pool_count_estimate svar, så venter man lige lidt ekstra, så hvis en drone kommer online igen, så kan den nå at være med.
                                                # Det kræver også hver drone har et unikt ID, så vi kan se hvem svarer
CHECK_ALL_CURRENT_PATHS_EXTRA_TIME = 1 # seconds. Extra time to wait after all estimated_neighbor_count responses are recieved (if estimated_neighbor_count < max_neighbor_count)
                                         #          This allows for drones that might come online again to be included. (and estimated_neighbor_count is then adjusted accordingly)
                                         #          (This setup only really makes sense if CHECK_ALL_CURRENT_PATHS_EXTRA_TIME < CHECK_ALL_CURRENT_PATHS_TIMEOUT)
                                         #          (for more info regarding this setup, see comments further down in this file)
        


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
        self.visited_cell_publisher = self.create_publisher(
            Cell, '/announce_visited_cell', qos_profile) # global topic, all drones publish and subscribe to the same topic (i.e. no individual drone namespace)

        # Create subscribers
        self.visited_cell_subscriber = self.create_subscription(
            Cell, '/announce_visited_cell', self.visited_cell_callback, qos_profile)

        # Create services
        self.sync_visited_cells_service = self.create_service(SyncVisitedCells, self.ns_drone + '/sync_visited_cells', self.sync_visited_cells_server_callback)
        self.check_all_current_paths_service = self.create_service(CheckAllCurrentPaths, self.ns_drone + '/check_current_path', self.check_all_current_paths_server_callback)

        # Create service clients
        # (for each service, we need a client for each drone)

        self.sync_visited_cells_clients = [] 
        for drone_id in range(1, self.max_drone_count + 1): # start from 1 to max_drone_count (inclusive)
            if drone_id == self.instance_id:
                continue # skip self
            drone_ns = "drone_" + str(drone_id)
            self.sync_visited_cells_clients.append(self.create_client(SyncVisitedCells, self.ns_launch +"/"+ drone_ns + '/sync_visited_cells'))
            while not self.sync_visited_cells_clients[-1].wait_for_service(timeout_sec=1.0): # (-1 gets the last added client)
                self.get_logger().info('service not available, waiting again...')
                
        self.check_all_current_paths_clients = []
        for drone_id in range(1, self.max_drone_count + 1): # start from 1 to max_drone_count (inclusive)
            if drone_id == self.instance_id:
                continue # skip self
            drone_ns = "drone_" + str(drone_id)
            self.check_all_current_paths_clients.append(self.create_client(CheckAllCurrentPaths, self.ns_launch +"/"+ drone_ns + '/check_current_path'))
            while not self.check_all_current_paths_clients[-1].wait_for_service(timeout_sec=1.0): # (-1 gets the last added client)
                self.get_logger().info('service not available, waiting again...')


        # Initialize  variables for communication
    


        ############ COMMUNICATION STUFF END ############ 
         
        

        # Read traversal order and home position from file (created in offline phase)
        pkl_path = os.path.join(os.path.join(os.path.dirname(__file__),'../','../','../','../','share', package_name, 'our_data'), 'offline_phase_data.pkl')
        with open(pkl_path, 'rb') as fp:
            data_loaded = pickle.load(fp) 
        # for item in os.listdir(dir_path):
        #     self.get_logger().info(f"Found item in parent dir: {item}")
        self.home_cell_from_offline = data_loaded['home_cell']
        self.home_gps_from_offline = data_loaded['home_gps']
        self.bf_traversal_cells = data_loaded['bf_traversal_cells']
        self.bf_traversal_gps = data_loaded['bf_traversal_gps']
        self.fly_nofly_grid = data_loaded['fly_nofly_grid']
        self.centroid = data_loaded['centroid'] # centroid of the Polygon area 
        self.bf_traversal_size = len(self.bf_traversal_gps)
        self.visited_cells = set() # Set to keep track of which cells have been visited (dont use "{}" because that creates an empty dict, not a set)
        
        # Initialize  variables
        self.one_sec_loop_count = int(1.0 / CONTROL_LOOP_DT)
        self.start_flight_delay_loops = int(float(self.start_flight_delay_s) / CONTROL_LOOP_DT)
        self.offboard_startup_counter = 0

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.vehicle_global_position = VehicleGlobalPosition()
        self.home_pos = HomePosition()

        self.map_projection_initialized = False

        self.path_clear = None # flag to indicate if path is clear (i.e. no other drones are in the way)
        self.path_clear_response_count = 0 # counter to keep track of how many drones we have checked the path with
        self.max_neighbor_count = self.max_drone_count - 1 # used for path checking
        self.estimated_neighbor_count = self.max_neighbor_count # used for path checking

        self.at_path_conflict_alt = False # Flag to indicate if we are at the increased altitude to avoid path conflict.
                                          # This setup only works if two drones are involved in the path conflict. A solution to handle path conflicts involving an arbitrary number of drones would make the implementation quite a bit more complex.

        # Create a main loop timer
        self.main_loop_timer = self.create_timer(CONTROL_LOOP_DT, self.control_loop_callback)

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
            if not (abs(home_position.lat - self.home_gps_from_offline[0]) < HOME_POS_TOLERANCE and abs(home_position.lon - self.home_gps_from_offline[1]) < HOME_POS_TOLERANCE):
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

    # Check All Current Paths setup in short:
        # When we want to check if our path is clear, we call check_all_current_paths()
        # This sends a request to all other drones, asking for their current path (from current position to current target)
        # Each drone responds with its current path
        # We then check if our path intersects with any of the received paths
        # If we find an intersection, we set self.path_clear = False
        # If we receive responses from all drones and find no intersections, we set self.path_clear = True
        # If we do not receive responses from all drones within a timeout period, we assume the path is clear (self.path_clear = True), but we adjust our estimated_neighbor_count down accordingly.
        # If we receive more responses than our estimated_neighbor_count, we adjust our estimated_neighbor_count accordingly (up to max_neighbor_count)
        # If our estimated_neighbor_count is less than max_neighbor_count, we add an extra wait time after receiving all responses, to allow for drones that might come online again to be included.

    # will set self.path_clear (to something other than None) when it completes (also, self.at_path_conflict_alt will be set to signal if we need to increase altitude or not)
    def check_all_current_paths(self):
        self.path_clear = None # reset flag
        self.path_clear_response_count = 0 # reset counter. (counter to keep track of how many drones we have checked the path with)

        for client in self.check_all_current_paths_clients:
            #self.get_logger().info(f"Requesting current path from another drone...")
            req = CheckAllCurrentPaths.Request()
            future = client.call_async(req)
            future.add_done_callback(self.check_all_current_paths_client_callback)

        # Start a timeout timer
        self.check_paths_timeout_timer = self.create_timer(CHECK_ALL_CURRENT_PATHS_TIMEOUT, self.check_all_current_paths_timeout)

    def check_all_current_paths_client_callback(self, future):

        if self.path_clear != None:
            # we have already decided if the path are clear or not! ignore all further responses for this check (path_clear i reset to None at next path check)
            # (timeout has already been cancelled at this point)
            return 

        if future.result() is not None:
            response = future.result()
            response_path = response.current_path
            response_at_path_conflict_alt = response.at_path_conflict_alt

            # Keep track of how many drones have responded
            self.path_clear_response_count += 1

            # Adjust estimated neighbor count if we receive more responses than expected (up to a max of max_neighbor_count)
            #self.get_logger().info(f"estimated_neighbor_count: {self.estimated_neighbor_count}, path_clear_response_count: {self.path_clear_response_count}")
            if self.path_clear_response_count > self.estimated_neighbor_count:
                #self.get_logger().warn(f"Received more path responses ({self.path_clear_response_count}) than estimated neighbor count ({self.estimated_neighbor_count}). This should not happen.")
                self.get_logger().info(f"MONSTER")
                self.estimated_neighbor_count = self.path_clear_response_count if self.path_clear_response_count < self.max_neighbor_count else self.max_neighbor_count # adjust estimated neighbor count to match reality
                return

            # Check if our current path intersects with the received path
            if tsunami_does_paths_cross(
                (self.vehicle_global_position.lat, self.vehicle_global_position.lon),
                (self.lat_target, self.lon_target),
                (response_path.from_lat, response_path.from_lon),
                (response_path.to_lat, response_path.to_lon)
            ):
                self.get_logger().info(f"Path conflict detected with another drone: from ({response_path.from_lat}, {response_path.from_lon}) to ({response_path.to_lat}, {response_path.to_lon})")
                self.path_clear = False
                # If the other drone is at an increased altitude to avoid a path conflict, we assume it is avoiding us, so we dont need to increase our altitude:
                if(response_at_path_conflict_alt == False):
                    #self.get_logger().info(f"SUSHI 1")
                    self.at_path_conflict_alt = True 
                else:
                    #self.get_logger().info(f"SUSHI 2")
                    self.at_path_conflict_alt = False 
                self.check_paths_timeout_timer.cancel() # stop the timeout timer - we already know the path is not clear
            else:
                #self.get_logger().info(f"No path conflict with another drone: from ({self.vehicle_global_position.lat}, {self.vehicle_global_position.lon}) to ({self.lat_target}, {self.lon_target})")
                
                if self.path_clear_response_count == self.estimated_neighbor_count:
                    if self.estimated_neighbor_count != self.max_neighbor_count:
                        # add some extra time to allow for drones that might come online again
                        self.get_logger().info(f"All drones responded, but estimated neighbor count ({self.estimated_neighbor_count}) is less than max neighbor count ({self.max_neighbor_count}). Adding extra wait time to allow for drones that might come online again.")
                        self.check_paths_timeout_timer.cancel() # stop the current timeout timer
                        self.check_paths_extra_timer = self.create_timer(CHECK_ALL_CURRENT_PATHS_EXTRA_TIME, self.check_all_current_paths_extra_time)
                    else :
                        self.path_clear = True # path is clear if we have checked with all other drones and found no conflicts
                        #self.get_logger().info(f"SUSHI 3")
                        self.at_path_conflict_alt = False # reset flag
                        self.check_paths_timeout_timer.cancel() # stop the timeout timer - we already know the path is clear

    def check_all_current_paths_timeout(self):
        #self.get_logger().warn("Request for all current paths timed out. (might be due to some drones being offline)")
        # we want the timer to be oneshot:
        self.check_paths_timeout_timer.cancel()

        # How many responses do we miss?
        missing_responses = self.estimated_neighbor_count - self.path_clear_response_count
        self.estimated_neighbor_count -= missing_responses # adjust estimated neighbor count to match reality

        self.path_clear = True # after timeout, we assume the path is clear (even though we are not 100% sure) - a drone might have gone offline since last check
        #self.get_logger().info(f"SUSHI 4")
        self.at_path_conflict_alt = False # reset flag

    def check_all_current_paths_extra_time(self):
        # we want the timer to be oneshot:
        self.check_paths_extra_timer.cancel()
        #self.get_logger().info("Extra wait time after all estimated drones responded is over.")

        # after the extra time, we assume the path is clear (even though less than max_neighbor_count drones responded)
        self.path_clear = True
        #self.get_logger().info(f"SUSHI 5")
        self.at_path_conflict_alt = False # reset flag

    def check_all_current_paths_server_callback(self, request, response):
        # this is called when another drone requests our current path
        response.current_path = Path(
            from_lat=self.vehicle_global_position.lat,
            from_lon=self.vehicle_global_position.lon,
            to_lat=self.lat_target,
            to_lon=self.lon_target,
        )
        response.at_path_conflict_alt = self.at_path_conflict_alt
        return response
        





    def broadcast_visited_cell(self, cell): # Publish Topic (its called broadcast for generilization with the tsunami implimentation for real hardware)
        # Broadcast that we have visited a cell by publishing to a topic
        #self.get_logger().info(f"Broadcasting visited cell {cell}")
        msg = Cell()
        msg.x = cell[0]
        msg.y = cell[1]
        self.visited_cell_publisher.publish(msg)


    def visited_cell_callback(self, cell): # Topic subscriber callback
        # This is called when we receive a broadcast from another drone that it has visited a cell
        #self.get_logger().info(f"Received visited cell {cell}. Marking as visited locally.")
        self.visited_cells.add((cell.x, cell.y))


    def sync_visited_cells(self): # service client
        # Request the "visited_cells" from all other drones, and update our own local set accordingly.
        # Calling this method at the start, will allow for late-joining drones to get the current state

        self.get_logger().info("Syncing visited cells with all other drones...")

        for client in self.sync_visited_cells_clients:
            req = SyncVisitedCells.Request()

            future = client.call_async(req)
            # Note: dont use .spin_until_future_complete here! it will deadlock! Because sync_visited_cells() is being run from the main controll loop (i.e. a timer callback!)
            # PROBLEM: Yes we use call_async(), but .spin_until_future_complete is blocking its asociated callback group! - and callbacks are needed to process the response. BUT callbacks cant be called because we are already in a timer callback and all the callbacks are in the same callback group!
            #          For more info: see https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html (they explain the problem very good)
            # SOLUTION 1: Use MultiThreadedExecutor and assign the callbacks different callback groups (by default, they all use the same). Then its allowed to block the callback!
            # SOLUTION 2 (our choice): Use call_async() and setup a callback for when the future is done (i.e. completety non-blocking and asyncronoues)

            future.add_done_callback(self.sync_visited_cells_client_future_callback)

    def sync_visited_cells_client_future_callback(self, future):
        
        if future.result() is not None:
            response = future.result()
            # Add visited cells to our local visited_cells set
            for cell in response.visited_cells: 
                self.visited_cells.add((cell.x, cell.y))

            #self.visited_waypoints = [a or b for a, b in zip(self.visited_waypoints, response.visited_waypoints)]
        else:
            self.get_logger().warn('Failed to receive visited cells. (Future is None)')

    def sync_visited_cells_server_callback(self, request, response): # service server callback
        # this is called when another drone requests our visited_cells list
        visited_cells_msg = [Cell(x=x, y=y) for x, y in self.visited_cells]
        response.visited_cells = visited_cells_msg
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
            self.sync_visited_cells() # Sync visited cells with all other drones
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
