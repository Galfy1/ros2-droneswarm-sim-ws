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
# ENABLE_SPLINE_INTERPOLATION = True # False: Fly directly to each waypoint. less compute, but less smooth flight. True: use spline interpolation to create a smooth path between waypoints.
#                                     # Tsunami paper calls for some kind of "trajectory" (aka interpolation) to be used. 
# SPLINE_RESOLUTION = 5  # number of interpolated points between each pair of waypoints. higher = smoother, but more compute. Ignored if ENABLE_SPLINE_INTERPOLATION is False.
WAYPOINT_REACHED_TOLERANCE = 1.0  # meters, how close we need to be to a waypoint to consider it "reached"


# TODO, her, i stedet for at pulibsh waypointet direkte, skal vi lave en spline interpolation, så dronen flyver glatt. aka en liste af waypoints den skal igennem for at komme til target waypoint
#       her skal den også have lidt tollerence, så den ikke laver små justeringer hele tiden
#       koden nedenfor (det med distancen) skal derfor også laves lidt om.
#       det betyder måske også at yaw beregningen beregnes om, hvor hver punkti splinen

# TODO vi kan bruge Qgroundcontrol pathen den tegner, til at se hvor dronen rent faktisk flyver hen


def great_circle_bearing(lat1, lon1, lat2, lon2):
    # (see https://www.movable-type.co.uk/scripts/latlong.html) ("Bearing" is the angle in Lat/Lon language)
    d_lon = lon2 - lon1
    term_1 = np.sin(d_lon) * np.cos(lat2)
    term_2 = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(d_lon)
    return np.arctan2(term_1, term_2)

def great_circle_distance(lat1, lon1, lat2, lon2):
    return haversine((lat1, lon1), (lat2, lon2), unit=Unit.METERS) # haversine is also called "Great-circle Distance Formula"


def tsunami_online_init(self):
    self.initial_alt_reached = False
    # self.start_pos_grappeded = False
    self.traversal_index = 0 # main index in traversal order
    self.traversal_index_commited = None # index in traversal order which is currently commited to be visited by this drone (i.e. reserved for this drone)
    self.visited_waypoints = [False] * self.traversal_order_size # keep track of which waypoints have been visited
    self.get_logger().info("Tsunami online initialized")



def all_waypoints_visited(self):
    self.get_logger().info("Completed all waypoints.")
    pass # TODO

def commit_to_new_waypoint(self, waypoint_index):
    # Locally reserve this waypoint as "currently being visited" by this drone:
    self.traversal_index_commited = waypoint_index 
    # Set waypoint as "visited" (we do it just as we start flying to it, to tell other drones that it is "reserved" by us):
    self.visited_waypoints[waypoint_index] = True # mark waypoint as visited locally
    self.broadcast_visited_waypoint(waypoint_index) # mark waypoint as visited for other drones


def tsunami_online_loop(self):


    # Wait until drone have reached  operating altitude
    if self.vehicle_local_position.z > OPERATING_ALTITUDE+0.5 and not self.initial_alt_reached:  # (remember NED coordinates: down is positive) (0.5m tolerance)
        self.publish_position_setpoint_global(self.home_pos.lat, self.home_pos.lon, OPERATING_ALTITUDE, OPERATING_VELOCITY)
        return
    else:
        self.initial_alt_reached = True


    # Skip to next waypoint if it is already visited (and not the one we are already commited to)
    while self.visited_waypoints[self.traversal_index] and not (self.traversal_index == self.traversal_index_commited) :
        if self.traversal_index >= self.traversal_order_size - 1:  # make sure we dont go out of bounds
            all_waypoints_visited(self)
            return
        self.traversal_index += 1
    
    if self.traversal_index != self.traversal_index_commited:
        # We have moved to a new waypoint, so we need to commit to it
        commit_to_new_waypoint(self, self.traversal_index)

    # Get current target waypoint coordinates
    lat_target, lon_target = self.traversal_order_gps[self.traversal_index_commited]

    # Calculate yaw to target waypoint (if enabled)
    yaw_rad = 0.0
    if ENABLE_YAW_TURNING:
        yaw_rad = great_circle_bearing(self.vehicle_global_position.lat, self.vehicle_global_position.lon, lat_target, lon_target) # using the Great-circle Bearing Formula 

    # Publish position setpoint to target waypoint
    self.publish_position_setpoint_global(lat_target, lon_target, OPERATING_ALTITUDE, OPERATING_VELOCITY, yaw_rad)

    # Check if we have reached the target waypoint
    dist_to_target = great_circle_distance(self.vehicle_global_position.lat, self.vehicle_global_position.lon, lat_target, lon_target) # in meters
    if dist_to_target < WAYPOINT_REACHED_TOLERANCE:
        self.get_logger().info(f"Reached waypoint: {self.traversal_index_commited} at lat: {lat_target}, lon: {lon_target}")

        self.traversal_index_commited = None # we are no longer committed to this waypoint (we can get a new waypoint next loop)
        if self.traversal_index >= self.traversal_order_size - 1:
            all_waypoints_visited(self)
            return
        self.traversal_index += 1 # move to next waypoint in traversal order

