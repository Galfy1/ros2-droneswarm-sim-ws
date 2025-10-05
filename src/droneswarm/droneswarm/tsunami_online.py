import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# see https://docs.px4.io/main/en/ros2/px4_ros2_control_interface.html for topic interface descriptions
# TODO se https://docs.px4.io/v1.12/en/simulation/gazebo.html#set-world-location 
# TODO se https://docs.px4.io/main/en/sim_gazebo_gz/#set-custom-takeoff-location 
# TODO det ser ud til gps koordinater er defineret i maps filen: https://github.com/PX4/PX4-gazebo-models/blob/main/worlds/baylands.sdf (i bunden af filen)
    # det kan jeg eventuel bruge til at lave et polygon i mission planner!

# see https://github.dev/PX4/px4_msgs for message definitions
# TrajectorySetpoint is for setpoints in meters
# TODO PositionSetpoint seems to be setpoint for gps coordinates ????



CONTROL_LOOP_DT = 0.1  # seconds

class TsunamiOnline(Node):

    def __init__(self) -> None:
        super().__init__('tsunami_online')

        self.declare_parameter('instance_id', 0)
        self.instance_id = self.get_parameter('instance_id').get_parameter_value().integer_value
        self.ns = "/px4_" + str(self.instance_id) # Namespace for multiple vehicle instances

        ##################### PX4 SPECIFIC #####################

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  
        )
        # Create publishers
        # TODO
        # Create subscribers
        # TODO

        # Initialize PX4 specific variables
        # TODO

        ##################### GENERIC #####################

        # Read traversal order from file (created in offline phase)
        with open('traversal_order_gps.pkl', 'rb') as fp:
            traversal_order_gps = pickle.load(fp)

        # Create a timer to publish control commands
        # TODO

        # Initialize generic variables


    ##################### PX4 SPECIFIC METHODS #####################

    def vehicle_local_position_callback(self, vehicle_local_position):
        # TODO 

    def vehicle_status_callback(self, vehicle_status):
        # TODO

    def arm(self):
        # TODO

    def disarm(self):
        # TODO

    def engage_offboard_mode(self):
        # TODO

    def land(self):
        # TODO

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        # TODO

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
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



    ##################### GENERIC METHODS #####################

    # TODO


    ##################### MAIN CONTROL LOOP #####################


    def controll_loop_callback(self) -> None:

        # TODO





def main(args=None) -> None:
    print('Starting Tsunami Online...')
    rclpy.init(args=args)

    tsunami_online = TsunamiOnline()
    rclpy.spin(tsunami_online)

    tsunami_online.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)