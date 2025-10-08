import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus



CONTROL_LOOP_DT = 0.1  # seconds

class OffboardControl(Node):

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        self.declare_parameter('instance_id', 0)

        self.instance_id = self.get_parameter('instance_id').get_parameter_value().integer_value
        self.ns = "/px4_" + str(self.instance_id) # Namespace for multiple vehicle instances

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # TODO Den her er 0 i community python eksemplet
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

        # Create a timer to publish control commands
        self.timer = self.create_timer(CONTROL_LOOP_DT, self.controll_loop_callback)

        # Initialize variables
        self.one_sec_loop_count = int(1.0 / CONTROL_LOOP_DT)
        self.offboard_startup_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0


    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
        #self.get_logger().info(f"Vehicle Local Position: {self.vehicle_local_position}")

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status
        #self.get_logger().info(f"Vehicle Status: {self.vehicle_status}")


    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Landing vehicle")

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
        """Publish the trajectory setpoint."""
        # see https://docs.px4.io/main/en/flight_modes/offboard.html for more info regarding trajectory setpoint
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        #self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

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


    def controll_loop_callback(self) -> None:

        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_startup_counter == self.one_sec_loop_count:
            self.engage_offboard_mode()
            self.arm()
            self.offboard_startup_counter = 9999  # Prevent re-entering this if statement

        # If drone is set to offboard mode and has not yet reached takeoff height, publish setpoints
        if self.vehicle_local_position.z > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)

        # Once drone has reached takeoff height, land and exit
        elif self.vehicle_local_position.z <= self.takeoff_height:
            self.land()
            exit(0) # TODO not sure what this does 

        if self.offboard_startup_counter < self.one_sec_loop_count:
            self.offboard_startup_counter += 1

        #self.get_logger().info("offboard_startup_counter: {self.offboard_startup_counter}")






def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)

    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)