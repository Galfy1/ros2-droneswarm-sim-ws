



# testy:
test_latitude = 37.412833
test_longitude = -121.998055




# ADJUSTABLE PARAMETERS
OPERATING_ALTITUDE = -3.0  # meters
OPERATING_SPEED = 2.0 # m/s



def tsunami_online_init(self):
    self.initial_alt_reached = False
    # self.start_pos_grappeded = False


def tsunami_online_loop(self):

    # # Grab start position once
    # if not self.start_pos_grappeded:
    #     self.start_pos = self.vehicle_local # take copy of current local position
    #     self.start_pos_grappeded = True

    #self.publish_position_setpoint_global(self.home_pos.lat, self.home_pos.lon, OPERATING_ALTITUDE, OPERATING_SPEED)
    self.publish_position_setpoint_local(0.0, 0.0, OPERATING_ALTITUDE, OPERATING_SPEED)


    # # Wait until drone have reached  operating altitude
    # if self.vehicle_local_position.z > OPERATING_ALTITUDE + 0.5 and not self.initial_alt_reached:  # (remember NED coordinates: down is positive) (0.5m tolerance)
    #     self.publish_position_setpoint_global(self.home_pos.lat, self.home_pos.lon, OPERATING_ALTITUDE, OPERATING_SPEED)
    #     return
    # else:
    #     self.initial_alt_reached = True


    # for lat, long in self.traversal_order_gps:
    #     self.get_logger().info(f"Flying to waypoint {lat}, {long}")
    #     self.publish_position_setpoint_global(lat, long, OPERATING_ALTITUDE, OPERATING_SPEED)
    #     # Wait until the drone is within 1 meter of the target waypoint
    #     while True:
    #         if self.vehicle_local.xy_valid and self.vehicle_local.z_valid:
    #             x_target, y_target = self.map_projection.global_to_local(lat, long)
    #             distance = ((self.vehicle_local.x - x_target) ** 2 + (self.vehicle_local.y - y_target) ** 2) ** 0.5
    #             if distance < 1.0:  # within 1 meter
    #                 break
    #         self.publish_position_setpoint_global(lat, long, OPERATING_ALTITUDE, OPERATING_SPEED)
    #         rclpy.spin_once(self)
    #         #self.get_logger().info(f"Distance to waypoint: {distance:.2f} meters")
 

    # lat, long = self.traversal_order_gps[30]  # first waypoint in traversal order
    # #x, y = self.map_projection.global_to_local(lat, long)
    # x, y = self.map_projection.global_to_local(test_latitude, test_longitude)

    # self.publish_position_setpoint(x, y, -5.0)