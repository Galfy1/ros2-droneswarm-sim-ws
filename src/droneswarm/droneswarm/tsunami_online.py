from haversine import haversine, Unit
import numpy as np
import math

# See the following for helpfull lat/lon/bearing calculations:
#    https://www.movable-type.co.uk/scripts/latlong.html  # TODO TILFØJ HAVERSINE OG BEARING BEREGNINGER TIL RAPPORTEN

# TODO skriv i rapport at vores version af Tsunami online er mere distributed (forklar det med at vi ikke har en drone pool..)
# TODO Fra tsunami paper "Notably, due to the properties of the waypoint grid constructed in the offline phase, multiple
#       points are often equally close to δω . Tsunami explores Ωin order to find the first such minimum point."
#       Det her er en ting vi har flyttet til offline phase.. men det burde ikke gøre en logisk forskel

# # testy:
# test_latitude = 37.412833
# test_longitude = -121.998055


# TODO vi skal have lavet det der spline halløj

# ADJUSTABLE PARAMETERS
OPERATING_ALTITUDE = -20.0  # meters (remeber, NED coordinates: down is positive)
OPERATING_VELOCITY = 1.0 # m/s # TODO DOES NOT CURRENTLY WORK IN THE PX4 SIM
ALTITUDE_TOLERENCE = 0.5  # meters, how close we need to be to the operating altitude to consider it "reached"
ENABLE_YAW_TURNING = True  # our real-world drone only has a 1D gimbal (pitch), so we want to turn the drone to face the direction of travel
# ENABLE_SPLINE_INTERPOLATION = True # False: Fly directly to each waypoint. less compute, but less smooth flight. True: use spline interpolation to create a smooth path between waypoints.
#                                     # Tsunami paper calls for some kind of "trajectory" (aka interpolation) to be used. 
# SPLINE_RESOLUTION = 5  # number of interpolated points between each pair of waypoints. higher = smoother, but more compute. Ignored if ENABLE_SPLINE_INTERPOLATION is False.
ENABLE_PATH_CONFLICT_CHECK = False  # if True, the drone will check if its path conflicts with other drones and adjust altitude if needed
WAYPOINT_REACHED_TOLERANCE = 1.0  # meters, how close we need to be to a waypoint to consider it "reached"
ALLOW_DIAGONAL_PATH_PLANNING = False  # if True, diagonal neighbors are considered neighbors when finding the next cell to visit. If False, only N/S/E/W neighbors are considered.
ALTITUDE_INCREASE_ON_PATH_CONFLICT = 5.0 # meters, how much to increase altitude if a path conflict is detected with another drone


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
    self.current_target_cell = self.home_cell_from_offline # initialize to home cell
    self.lat_target, self.lon_target = cell_to_gps(self, self.current_target_cell) # get initial target gps coordinates
    self.flight_path_log = [(float(self.lat_target), float(self.lon_target))]  # list to log the flight path (lat, lon) tuples - for analysis later
    self.flight_complete = False 
    self.waiting_for_path_check = False 
    self.get_logger().info("Tsunami online initialized")


def all_cells_visited(self):
    #self.get_logger().info("Completed all cells.")

    # Print the flight path log
    if self.flight_complete:
        return # already completed

    # Return to home position and land:
    #self.get_logger().info("Returning to home position and landing.")

    self.publish_position_setpoint_global(self.home_pos.lat, self.home_pos.lon, OPERATING_ALTITUDE, OPERATING_VELOCITY) # go to home position at ground level

    # TROR DEN DRIFTER TIL SIDEN NÅR DEN LANDER HVIS DEN IKKE ER HELT STILLE.
    # # Check if we have reached home position:
    # dist_to_home = great_circle_distance(self.vehicle_global_position.lat, self.vehicle_global_position.lon, self.home_pos.lat, self.home_pos.lon) # in meters
    # if dist_to_home < WAYPOINT_REACHED_TOLERANCE:
    #     self.get_logger().info("Reached home position, landing now.")
    #     self.land()
    #     self.get_logger().info(f"FLIGHT PATH LOG: {self.flight_path_log}")
    #     self.flight_complete = True 

    pass # TODO



# def tsunami_online_init(self):
#     self.initial_alt_reached = False
#     self.traversal_index = 0 # main index in traversal order
#     self.traversal_index_commited = None # index in traversal order which is currently commited to be visited by this drone (i.e. reserved for this drone)
#     self.get_logger().info("Tsunami online initialized")



# def all_waypoints_visited(self):
#     self.get_logger().info("Completed all waypoints.")
#     pass # TODO

# def commit_to_new_waypoint(self, waypoint_index):
#     # Locally reserve this waypoint as "currently being visited" by this drone:
#     self.traversal_index_commited = waypoint_index 
#     # Set waypoint as "visited" (we do it just as we start flying to it, to tell other drones that it is "reserved" by us):
#     self.visited_waypoints[waypoint_index] = True # mark waypoint as visited locally
#     self.broadcast_visited_waypoint(waypoint_index) # mark waypoint as visited for other drones



def tsunami_does_paths_cross(path1_from, path1_to, path2_from, path2_to):
    # # Check if two line segments (paths) cross each other
    # # Each path is defined by a "from" and "to" GPS coordinate (lat, lon)
    # # Using the algorithm from https://stackoverflow.com/a/1968345 (from "Tricks of the Windows Game Programming Gurus" by André LaMothe, 2002)




    """
    Returns (True, (ix, iy)) if lines p0-p1 and p2-p3 intersect, otherwise (False, None).
    Each point is a tuple (x, y).
    """
    # extract the 4 points (coordinates)
    p0_x, p0_y = path1_from
    p1_x, p1_y = path1_to
    p2_x, p2_y = path2_from
    p3_x, p3_y = path2_to

    # Calculate the direction vectors of the lines
    s1_x = p1_x - p0_x
    s1_y = p1_y - p0_y
    s2_x = p3_x - p2_x
    s2_y = p3_y - p2_y

    denom = (-s2_x * s1_y + s1_x * s2_y)
    if denom == 0:
        return False #, None  # Parallel or coincident lines

    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / denom
    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / denom

    if 0 <= s <= 1 and 0 <= t <= 1:
        # Intersection point
        # ix = p0_x + (t * s1_x)
        # iy = p0_y + (t * s1_y)
        return True #, (ix, iy)

    return False #, None










    # def orientation(p, q, r):
    #     val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    #     if val == 0:
    #         return 0  # collinear
    #     return 1 if val > 0 else 2  # clock or counterclock wise

    # def on_segment(p, q, r):
    #     if (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and
    #             q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1])):
    #         return True
    #     return False

    # p1 = (path1_from[0], path1_from[1])
    # q1 = (path1_to[0], path1_to[1])
    # p2 = (path2_from[0], path2_from[1])
    # q2 = (path2_to[0], path2_to[1])

    # o1 = orientation(p1, q1, p2)
    # o2 = orientation(p1, q1, q2)
    # o3 = orientation(p2, q2, p1)
    # o4 = orientation(p2, q2, q1)

    # # General case
    # if o1 != o2 and o3 != o4:
    #     return True

    # # Special Cases
    # # p1, q1 and p2 are collinear and p2 lies on segment p1q1
    # if o1 == 0 and on_segment(p1, p2, q1):
    #     return True

    # # p1, q1 and p2 are collinear and q2 lies on segment p1q1
    # if o2 == 0 and on_segment(p1, q2, q1):
    #     return True

    # # p2, q2 and p1 are collinear and p1 lies on segment p2q2
    # if o3 == 0 and on_segment(p2, p1, q2):
    #     return True


# Find the next cell in the Breadth First Traversal
# bft_cells: list of (x,y) tuples
# current_cell: (x,y) tuple
# visited_cells: set of (x,y) tuples
# allow_diagonal: bool, if True, diagonal neighbors are considered neighbors
def find_next_cell(bft_cells, current_cell, visited_cells, allow_diagonal=False):

    # Find current location in bft_cells
    if current_cell not in bft_cells:
        raise ValueError("Current cell is not in the list of BFT cells.")
    current_index = bft_cells.index(current_cell)

    # Find the next cell in the BFT order that is a neighbor of the current cell (and not yet visited)
    for i in range(current_index + 1, len(bft_cells)):
        next_cell = bft_cells[i]
        if next_cell not in visited_cells:
            dx = abs(next_cell[0] - current_cell[0])
            dy = abs(next_cell[1] - current_cell[1])

            if (allow_diagonal and max(dx, dy) == 1) or (not allow_diagonal and dx + dy == 1):
                # neighbor found!
                return next_cell


    # No unvisited neighbor is found. Find the closest unvisited cell
    min_dist = float("inf")
    closest_cell = None
    for cell in bft_cells:
        if cell not in visited_cells:
            dx = abs(cell[0] - current_cell[0])
            dy = abs(cell[1] - current_cell[1])
            dist = math.sqrt(dx**2 + dy**2)  # Euclidean distance
            if dist < min_dist:
                min_dist = dist
                closest_cell = cell
    return closest_cell


def update_target_cell(self):
    # Find the next cell in the Breadth First Traversal
    next_cell = find_next_cell(self.bf_traversal_cells, self.current_target_cell, self.visited_cells, ALLOW_DIAGONAL_PATH_PLANNING)
    if next_cell is not None:
        # New target cell found!
        self.current_target_cell = next_cell
        self.lat_target, self.lon_target = cell_to_gps(self, next_cell)
        # Mark the next cell as "visited" (both locally and broadcast to other drones) - this is to "reserve" the cell for this drone
        self.visited_cells.add(next_cell)
        self.broadcast_visited_cell(next_cell) 
        self.flight_path_log.append((float(self.lat_target), float(self.lon_target))) # log the flight path
    else:
        self.get_logger().info("No valid next cell found.")
        # this means all cells have been visited
        all_cells_visited(self) 


def cell_to_gps(self, cell):
    
    # Find index of cell in bf
    cell_index = self.bf_traversal_cells.index(cell) # this should only result in one index, since each cell is unique in the traversal order

    # Convert cell to GPS coordinates
    gps_coords = self.bf_traversal_gps[cell_index]
    return gps_coords


def tsunami_online_loop(self):

 
    # Wait until drone have reached  operating altitude
    if self.vehicle_local_position.z > OPERATING_ALTITUDE+ALTITUDE_TOLERENCE and not self.initial_alt_reached:  # (remember NED coordinates: down is positive) (0.5m tolerance)
        self.publish_position_setpoint_global(self.home_pos.lat, self.home_pos.lon, OPERATING_ALTITUDE, OPERATING_VELOCITY)
        return
    else:
        self.initial_alt_reached = True

    # Check if all cells have been visited
    if len(self.visited_cells) >= self.bf_traversal_size:
        all_cells_visited(self)
        return

    # Check if we have reached the target cell - if so, update to next target cell
    dist_to_target = great_circle_distance(self.vehicle_global_position.lat, self.vehicle_global_position.lon, self.lat_target, self.lon_target) # in meters
    if dist_to_target < WAYPOINT_REACHED_TOLERANCE:
        self.get_logger().info(f"Reached cell: {self.current_target_cell} at lat: {self.lat_target}, lon: {self.lon_target}")
        update_target_cell(self) # Update to next target cell

    # Calculate yaw to target waypoint (if enabled)
    yaw_rad = 0.0
    if ENABLE_YAW_TURNING:
        yaw_rad = great_circle_bearing(self.vehicle_global_position.lat, self.vehicle_global_position.lon, self.lat_target, self.lon_target) # using the Great-circle Bearing Formula

    # Check if we need to increase altitude to avoid conflicting paths with other drones
    operating_altitude = OPERATING_ALTITUDE
    if ENABLE_PATH_CONFLICT_CHECK:
        if not self.waiting_for_path_check:
            self.check_all_current_paths() 
            self.waiting_for_path_check = True
        if self.path_clear == None:
            return # still waiting for path check to complete
        elif self.path_clear == False:
            #self.get_logger().info("Path not clear, increasing altitude to avoid conflict.")
            operating_altitude -= ALTITUDE_INCREASE_ON_PATH_CONFLICT # increase altitude by 5 meters to avoid conflict
            # wait until we have reached the increased altitude (within tolerance):
            if self.vehicle_local_position.z > operating_altitude + ALTITUDE_TOLERENCE:
                self.publish_position_setpoint_global(self.vehicle_global_position.lat, self.vehicle_global_position.lon, operating_altitude, OPERATING_VELOCITY)
                return 
        self.waiting_for_path_check = False

    # Publish position setpoint to target waypoint
    self.publish_position_setpoint_global(self.lat_target, self.lon_target, operating_altitude, OPERATING_VELOCITY, yaw_rad)

























    # # Wait until drone have reached  operating altitude
    # if self.vehicle_local_position.z > OPERATING_ALTITUDE+0.5 and not self.initial_alt_reached:  # (remember NED coordinates: down is positive) (0.5m tolerance)
    #     self.publish_position_setpoint_global(self.home_pos.lat, self.home_pos.lon, OPERATING_ALTITUDE, OPERATING_VELOCITY)
    #     return
    # else:
    #     self.initial_alt_reached = True


    # # Skip to next waypoint if it is already visited (and not the one we are already commited to)
    # while self.visited_waypoints[self.traversal_index] and not (self.traversal_index == self.traversal_index_commited) :
    #     if self.traversal_index >= self.traversal_order_size - 1:  # make sure we dont go out of bounds
    #         all_waypoints_visited(self)
    #         return
    #     self.traversal_index += 1
    
    # if self.traversal_index != self.traversal_index_commited:
    #     # We have moved to a new waypoint, so we need to commit to it
    #     commit_to_new_waypoint(self, self.traversal_index)

    # # Get current target waypoint coordinates
    # lat_target, lon_target = self.traversal_order_gps[self.traversal_index_commited]

    # # Calculate yaw to target waypoint (if enabled)
    # yaw_rad = 0.0
    # if ENABLE_YAW_TURNING:
    #     yaw_rad = great_circle_bearing(self.vehicle_global_position.lat, self.vehicle_global_position.lon, lat_target, lon_target) # using the Great-circle Bearing Formula 

    # # Publish position setpoint to target waypoint
    # self.publish_position_setpoint_global(lat_target, lon_target, OPERATING_ALTITUDE, OPERATING_VELOCITY, yaw_rad)

    # # Check if we have reached the target waypoint
    # dist_to_target = great_circle_distance(self.vehicle_global_position.lat, self.vehicle_global_position.lon, lat_target, lon_target) # in meters
    # if dist_to_target < WAYPOINT_REACHED_TOLERANCE:
    #     self.get_logger().info(f"Reached waypoint: {self.traversal_index_commited} at lat: {lat_target}, lon: {lon_target}")

    #     self.traversal_index_commited = None # we are no longer committed to this waypoint (we can get a new waypoint next loop)
    #     if self.traversal_index >= self.traversal_order_size - 1:
    #         all_waypoints_visited(self)v
    #         return
    #     self.traversal_index += 1 # move to next waypoint in traversal order

