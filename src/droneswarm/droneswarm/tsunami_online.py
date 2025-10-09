from haversine import haversine, Unit
import numpy as np

# See the following for helpfull lat/lon/bearing calculations:
#    https://www.movable-type.co.uk/scripts/latlong.html  # TODO TILFØJ HAVERSINE OG BEARING BEREGNINGER TIL RAPPORTEN

# testy:
test_latitude = 37.412833
test_longitude = -121.998055




# ADJUSTABLE PARAMETERS
OPERATING_ALTITUDE = -15.0  # meters
OPERATING_VELOCITY = 1.0 # m/s
ENABLE_YAW_TURNING = True  # our real-world drone only has a 1D gimbal (pitch), so we want to turn the drone to face the direction of travel


def great_circle_bearing(lat1, lon1, lat2, lon2):
    # (see https://www.movable-type.co.uk/scripts/latlong.html) ("Bearing" is the angle in Lat/Lon language)
    d_lon = lon2 - lon1
    term_1 = np.sin(d_lon) * np.cos(lat2)
    term_2 = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(d_lon)
    return np.arctan2(term_1, term_2)


def tsunami_online_init(self):
    self.initial_alt_reached = False
    # self.start_pos_grappeded = False
    self.traversal_index = 0

# TODO vi kan bruge Qgroundcontrol pathen den tegner, til at se hvor dronen rent faktisk flyver hen

def tsunami_online_loop(self):

    # # Grab start position once
    # if not self.start_pos_grappeded:
    #     self.start_pos = self.vehicle_local # take copy of current local position
    #     self.start_pos_grappeded = True

    #self.publish_position_setpoint_global(self.home_pos.lat, self.home_pos.lon, OPERATING_ALTITUDE, OPERATING_SPEED)
    #self.publish_position_setpoint_local(0.0, 0.0, OPERATING_ALTITUDE, OPERATING_SPEED)


    # Wait until drone have reached  operating altitude
    if self.vehicle_local_position.z > OPERATING_ALTITUDE+0.5 and not self.initial_alt_reached:  # (remember NED coordinates: down is positive) (0.5m tolerance)
        self.publish_position_setpoint_global(self.home_pos.lat, self.home_pos.lon, OPERATING_ALTITUDE, OPERATING_VELOCITY)
        return
    else:
        self.initial_alt_reached = True

    #self.publish_position_setpoint_global(test_latitude, test_longitude, OPERATING_ALTITUDE, OPERATING_VELOCITY)

    # get current target waypoint
    lat_target, lon_target = self.traversal_order_gps[self.traversal_index]


    yaw_rad = 0.0
    if ENABLE_YAW_TURNING:
        # Calculate yaw to next waypoint using the Great-circle Bearing Formula 
        yaw_rad = great_circle_bearing(self.vehicle_global_position.lat, self.vehicle_global_position.lon, lat_target, lon_target)



    self.publish_position_setpoint_global(*self.traversal_order_gps[self.traversal_index], OPERATING_ALTITUDE, OPERATING_VELOCITY, yaw_rad)

    # Check if we are within 1 meter of the target waypoint
    # self.get_logger().info(f"Flying to {lat_target}, {lon_target}")
    # self.get_logger().info(f"Current GPS: {self.vehicle_global_position.lat}, {self.vehicle_global_position.lon}")
    distance = haversine((self.vehicle_global_position.lat, self.vehicle_global_position.lon), (lat_target, lon_target), unit=Unit.METERS) # haversine is also called "Great-circle Distance Formula"
    #self.get_logger().info(f"Distance to waypoint {self.traversal_index+1}/{len(self.traversal_order_gps)}: {distance:.2f} meters")
    if distance < 1.0:  # within 1 meter
       # self.get_logger().info(f"Reached waypoint {self.traversal_index+1}/{len(self.traversal_order_gps)} at {lat_target}, {lon_target}")
        self.traversal_index += 1
        if self.traversal_index >= len(self.traversal_order_gps):
            self.get_logger().info("Completed all waypoints. Hovering at last position.")
            self.traversal_index = len(self.traversal_order_gps) - 1  # stay at last waypoint
    

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