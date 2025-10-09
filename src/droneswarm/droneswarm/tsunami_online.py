from haversine import haversine, Unit
import numpy as np

# See the following for helpfull lat/lon/bearing calculations:
#    https://www.movable-type.co.uk/scripts/latlong.html  # TODO TILFØJ HAVERSINE OG BEARING BEREGNINGER TIL RAPPORTEN

# TODO skriv i rapport at vores version af Tsunami online er mere distributed (forklar det med at vi ikke har en drone pool..)
# TODO Fra tsunami paper "Notably, due to the properties of the waypoint grid constructed in the offline phase, multiple
#       points are often equally close to δω . Tsunami explores Ωin order to find the first such minimum point."
#       Det her er en ting vi har flyttet til offline phase.. men det burde ikke gøre en logisk forskel

# testy:
test_latitude = 37.412833
test_longitude = -121.998055


# TODO vi skal have lavet det der spline halløj

# ADJUSTABLE PARAMETERS
OPERATING_ALTITUDE = -15.0  # meters
OPERATING_VELOCITY = 1.0 # m/s # TODO DOES NOT CURRENTLY WORK IN THE PX4 SIM
ENABLE_YAW_TURNING = True  # our real-world drone only has a 1D gimbal (pitch), so we want to turn the drone to face the direction of travel
ENABLE_SPLINE_INTERPOLATION = True # False: Fly directly to each waypoint. less compute, but less smooth flight. True: use spline interpolation to create a smooth path between waypoints.
                                    # Tsunami paper calls for some kind of "trajectory" (aka interpolation) to be used. 
SPLINE_RESOLUTION = 5  # number of interpolated points between each pair of waypoints. higher = smoother, but more compute. Ignored if ENABLE_SPLINE_INTERPOLATION is False.


def great_circle_bearing(lat1, lon1, lat2, lon2):
    # (see https://www.movable-type.co.uk/scripts/latlong.html) ("Bearing" is the angle in Lat/Lon language)
    d_lon = lon2 - lon1
    term_1 = np.sin(d_lon) * np.cos(lat2)
    term_2 = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(d_lon)
    return np.arctan2(term_1, term_2)

def great_cicle_distance(lat1, lon1, lat2, lon2):
    return haversine((lat1, lon1), (lat2, lon2), unit=Unit.METERS) # haversine is also called "Great-circle Distance Formula"


def tsunami_online_init(self, traversal_order_size):
    self.initial_alt_reached = False
    # self.start_pos_grappeded = False
    self.traversal_index = 0
    self.waypoints_visisted = [False] * traversal_order_size # keep track of which waypoints have been visited
    self.get_logger().info("Tsunami online initialized")

# TODO vi kan bruge Qgroundcontrol pathen den tegner, til at se hvor dronen rent faktisk flyver hen

def tsunami_online_loop(self):


    # Wait until drone have reached  operating altitude
    if self.vehicle_local_position.z > OPERATING_ALTITUDE+0.5 and not self.initial_alt_reached:  # (remember NED coordinates: down is positive) (0.5m tolerance)
        self.publish_position_setpoint_global(self.home_pos.lat, self.home_pos.lon, OPERATING_ALTITUDE, OPERATING_VELOCITY)
        return
    else:
        self.initial_alt_reached = True

    # get current target waypoint
    lat_target, lon_target = self.traversal_order_gps[self.traversal_index]

    yaw_rad = 0.0
    if ENABLE_YAW_TURNING:
        # Calculate yaw to next waypoint using the Great-circle Bearing Formula 
        yaw_rad = great_circle_bearing(self.vehicle_global_position.lat, self.vehicle_global_position.lon, lat_target, lon_target)


    # TODO, her, i stedet for at pulibsh waypointet direkte, skal vi lave en spline interpolation, så dronen flyver glatt. aka en liste af waypoints den skal igennem for at komme til target waypoint
    #       her skal den også have lidt tollerence, så den ikke laver små justeringer hele tiden
    #       koden nedenfor (det med distancen) skal derfor også laves lidt om.
    #       det betyder måske også at yaw beregningen beregnes om, hvor hver punkti splinen
    self.publish_position_setpoint_global(*self.traversal_order_gps[self.traversal_index], OPERATING_ALTITUDE, OPERATING_VELOCITY, yaw_rad)

    # Check if we are within 1 meter of the target waypoint
    # self.get_logger().info(f"Flying to {lat_target}, {lon_target}")
    # self.get_logger().info(f"Current GPS: {self.vehicle_global_position.lat}, {self.vehicle_global_position.lon}")
    dist_to_target = great_circle_distance(self.vehicle_global_position.lat, self.vehicle_global_position.lon, lat_target, lon_target) # in meters
    #self.get_logger().info(f"Distance to waypoint {self.traversal_index+1}/{len(self.traversal_order_gps)}: {distance:.2f} meters")
    if dist_to_target < 1.0:  # within 1 meter
       # self.get_logger().info(f"Reached waypoint {self.traversal_index+1}/{len(self.traversal_order_gps)} at {lat_target}, {lon_target}")
        self.traversal_index += 1
        if self.traversal_index >= len(self.traversal_order_gps):
            self.get_logger().info("Completed all waypoints. Hovering at last position.")
            self.traversal_index = len(self.traversal_order_gps) - 1  # stay at last waypoint


# TODO HUSK AT KIG PÅ TRAVERSAL ORDEREN OG SE OM DEN GIVER MENING (se det snakkede om i fitten)