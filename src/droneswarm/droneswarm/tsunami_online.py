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

PATH_PLANNING_METHOD = 'BFT'  # Options: "BFT","centroid", "hybrid"
ALLOW_DIAGONAL_PATH_PLANNING = False  # if True, diagonal neighbors are considered neighbors when finding the next cell to visit. If False, only N/S/E/W neighbors are considered.

OPERATING_ALTITUDE = -25.0  # meters (remeber, NED coordinates: down is positive)
OPERATING_VELOCITY = 1.0 # m/s # TODO DOES NOT CURRENTLY WORK IN THE PX4 SIM
ALTITUDE_TOLERENCE = 0.5  # meters, how close we need to be to the operating altitude to consider it "reached"
ENABLE_YAW_TURNING = True  # our real-world drone only has a 1D gimbal (pitch), so we want to turn the drone to face the direction of travel
# ENABLE_SPLINE_INTERPOLATION = True # False: Fly directly to each waypoint. less compute, but less smooth flight. True: use spline interpolation to create a smooth path between waypoints.
#                                     # Tsunami paper calls for some kind of "trajectory" (aka interpolation) to be used. 
# SPLINE_RESOLUTION = 5  # number of interpolated points between each pair of waypoints. higher = smoother, but more compute. Ignored if ENABLE_SPLINE_INTERPOLATION is False.
ENABLE_PATH_CONFLICT_CHECK = True  # if True, the drone will check if its path conflicts with other drones and adjust altitude if needed
WAYPOINT_REACHED_TOLERANCE = 1.0  # meters, how close we need to be to a waypoint to consider it "reached"

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
    self.trigger_path_check = False
    self.waiting_for_path_check = False 
    self.land_stabelize_counter = 0
    self.operating_altitude = OPERATING_ALTITUDE # current operating altitude (can be increased to avoid path conflicts)

    self.get_logger().info("Tsunami online initialized")


def all_cells_visited(self):
    #self.get_logger().info("Completed all cells.")

    # Print the flight path log
    if self.flight_complete:
        return # already completed


    # TODO der er ingen path collision chechs mens vi returner hjem...

    # Return to home position and land:
    #self.get_logger().info("Returning to home position and landing.")
    # Calculate yaw to target waypoint (if enabled)

    self.lat_target = self.home_pos.lat 
    self.lon_target = self.home_pos.lon

    yaw_rad = 0.0
    if ENABLE_YAW_TURNING:
        yaw_rad = great_circle_bearing(self.vehicle_global_position.lat, self.vehicle_global_position.lon, self.lat_target, self.lon_target)

    self.publish_position_setpoint_global(self.home_pos.lat, self.home_pos.lon, self.operating_altitude, OPERATING_VELOCITY, yaw_rad) # go to home position at ground level

    # Check if we have reached home position:
    dist_to_home = great_circle_distance(self.vehicle_global_position.lat, self.vehicle_global_position.lon, self.home_pos.lat, self.home_pos.lon) # in meters
    if dist_to_home < 0.3:  # within 30cm of home position

        # wait a bit to stabilize before landing:
        if self.land_stabelize_counter < self.one_sec_loop_count*2:
            self.land_stabelize_counter += 1
            return

        self.get_logger().info("Reached home position, landing now.")
        self.land() # TODO den lander altså ret skævt... har sat den til 2sec til stablizie... så test lige det
        self.get_logger().info(f"FLIGHT PATH LOG: {self.flight_path_log}")
        self.flight_complete = True 

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
def find_next_cell_bft(bft_cells, current_cell, visited_cells, allow_diagonal=False):

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


# Function to check if a cell
# is be visited or not
def _is_cell_valid(grid, vis, x, y):
  
    # If cell lies out of bounds
    if (x < 0 or y < 0 or x >= grid.shape[1] or y >= grid.shape[0]):
        return False

    # If cell is already visited
    if (vis[y][x]):
        return False
    
    # If cell is not traversable (i.e. "no fly zone")
    if (grid[y][x] == 0):
        return False

    # Otherwise
    return True

def _find_closest_cell(grid, current_cell, visited_cells): # TODO BFT INPLIMENTERINGE KAN NOK OGSÅ BARE BRUGE DEN HER?
    min_dist = float("inf")
    closest_cell = None
    for y in range(grid.shape[0]):
        for x in range(grid.shape[1]):
            if(_is_cell_valid(grid, visited_cells, x, y)):
                dx = abs(x - current_cell[1])
                dy = abs(y - current_cell[0])
                dist = math.sqrt(dx**2 + dy**2)  # Euclidean distance
                if dist < min_dist:
                    min_dist = dist
                    closest_cell = (y, x)
    return closest_cell # closest unvisited cell. returns None if no more valid cells are left


def _find_centroid_angle_diff_of_neighbors(grid, current_cell, visited_cells, centroid_line_angle: float, 
                                                 directional: str, allow_diagonal_in_path = True):
    #neighbor_with_smallest_angle_diff = None  # angle diff compared to centroid line direction
    x = current_cell[1]
    y = current_cell[0]
    result = []

    # TODO LIGE NU ER DET KUN 8 WAY. AKA allow_diagonal_in_path GØR INGENTING... IMPLIMENTER 4 WAY OGSÅ?

    for i in range(8):
        adjx = x + dRow_8way[i]
        adjy = y + dCol_8way[i]
        if (_is_cell_valid(grid, visited_cells, adjx, adjy)):
            neighbor_cell = (adjy, adjx)

            # Calculate angle from current_cell to neighbor_cell
            angle_to_neighbor = math.atan2(neighbor_cell[0] - current_cell[0], neighbor_cell[1] - current_cell[1])  # angle in radians

            # Calculate angle difference to centroid line angle
            
            angle_diff_rad = abs(angle_to_neighbor - centroid_line_angle)    # raw difference, but could be anywhere from 0 to 2π.
            angle_diff_rad = min(angle_diff_rad, 2*math.pi - angle_diff_rad) # ensure in [0, pi]. This step ensures we are measuring the shorter way around the circle (e.g. 350° → 10°).
            if directional == "bidirectional":
                angle_diff_rad = min(angle_diff_rad, math.pi - angle_diff_rad)   # ensure in [0, pi/2]. Folds any obtuse angle (>90°) back into an acute one, giving [0, pi/2].

            #print(f"Neighbor {i}, angle diff to centroid: {angle_diff_rad}")

            result.append((neighbor_cell, angle_diff_rad))

    return result  # list of (neighbor_cell, angle_diff_rad)

    #         # Check if this neighbor has the smallest angle difference so far
    #         if (neighbor_with_smallest_angle_diff is None) or (angle_diff_rad < neighbor_with_smallest_angle_diff[1]):
    #             neighbor_with_smallest_angle_diff = (neighbor_cell, angle_diff_rad)

    # return neighbor_with_smallest_angle_diff  # (neighbor_cell, angle_diff_rad) or None if no valid neighbor found


# "Unidirectional" angle difference (0 to pi). "Bidirectional" would be (0 to pi/2). 
# "Bidirectional" does not seem to work very well for the "pure centroid" method (constantly shifting direction)
def find_next_cell_centroid(grid, current_cell, visited_cells, centroid_line_angle: float, 
                             directional = "unidirectional", allow_diagonal_in_path = True):

    centroid_angle_diff_of_neighbors = _find_centroid_angle_diff_of_neighbors(grid, current_cell, visited_cells, centroid_line_angle, directional, allow_diagonal_in_path)

    if centroid_angle_diff_of_neighbors: # if list is not empty
        # Find the neighbor with the smallest angle difference
        # centroid_angle_diff_of_neighbors is a list of (neighbor_cell, angle_diff_rad)
        neighbor_with_smallest_angle_diff = min(centroid_angle_diff_of_neighbors, key=lambda x: x[1])
        return neighbor_with_smallest_angle_diff[0] 
    else:
        # No unvisited neighbor is found. Find the closest unvisited cell
        #print("No unvisited neighbor found. Finding closest unvisited cell...")
        return _find_closest_cell(grid, current_cell, visited_cells) # closest unvisited cell. returns None if no more valid cells are left




def update_target_cell(self):

    # TODO ENDNU EN ALTERNATIV STRAT TIL PATH PLANNING. (nem at implimentere), "centroid90" - hvor den i stedet flyver 90 grader til cetrnoid linjen

    # Find the next cell in the Breadth First Traversal
    if PATH_PLANNING_METHOD == 'BFT':
        next_cell = find_next_cell_bft(self.bf_traversal_cells, self.current_target_cell, self.visited_cells, ALLOW_DIAGONAL_PATH_PLANNING)
    elif PATH_PLANNING_METHOD == 'centroid':
        next_cell = find_next_cell_centroid(self.fly_nofly_grid, self.current_target_cell, self.visited_cells,
                                            self.centroid_line_angle, allow_diagonal_in_path=ALLOW_DIAGONAL_PATH_PLANNING)
    elif PATH_PLANNING_METHOD == 'hybrid':
        # TODO
        pass
    if next_cell is not None:
        # New target cell found!
        self.current_target_cell = next_cell
        self.lat_target, self.lon_target = cell_to_gps(self, next_cell)
        # Mark the next cell as "visited" (both locally and broadcast to other drones) - this is to "reserve" the cell for this drone
        self.visited_cells.add(next_cell)
        self.broadcast_visited_cell(next_cell) 
        self.flight_path_log.append((float(self.lat_target), float(self.lon_target))) # log the flight path for later plotting
    else:
        self.get_logger().info("No valid next cell found.")
        # this means all cells have been visited
        all_cells_visited(self) 


def cell_to_gps(self, cell):
    
    # Find index of cell in bf
    #cell_index = self.bf_traversal_cells.index(cell) # this should only result in one index, since each cell is unique in the traversal order
    cell_index = self.fly_nofly_grid.index(cell)

    # Convert cell to GPS coordinates
    #gps_coord = self.bf_traversal_gps[cell_index]
    gps_coord = self.fly_nofly_grid_gps[cell_index]
    return gps_coord


# Will move drone to target waypoint, while taking into acount path conflicts with other drones. It
# will return:  True if we are still in a path conflict situation (i.e. need to wait)
#               False if we are not in a path conflict situation or we have reached the target waypoint

# def path_conflict_avoidance(self):

#     if self.path_clear != False:
#         return False # we are not in a path conflict situation

#     # we now set our altitude (if we need to increase it or not)
#     operating_altitude = OPERATING_ALTITUDE
#     if self.at_path_conflict_alt == True:
#         operating_altitude = OPERATING_ALTITUDE - ALTITUDE_INCREASE_ON_PATH_CONFLICT

#     # make sure we are at the desired altitude
#     if self.vehicle_local_position.z > operating_altitude + ALTITUDE_TOLERENCE:
#         self.publish_position_setpoint_global(self.vehicle_global_position.lat, self.vehicle_global_position.lon, operating_altitude, OPERATING_VELOCITY)
#         return True # signal that we are still increasing altitude

#     # we are at the increased altitude! great! lets continues to the target waypoint at this altitude
#     self.publish_position_setpoint_global(self.lat_target, self.lon_target, operating_altitude, OPERATING_VELOCITY)

#     return ASD



def set_correct_operating_altitude(self):
    # we now set our altitude (if we need to increase it or not)
    if ENABLE_PATH_CONFLICT_CHECK and self.at_path_conflict_alt == True: 
        self.operating_altitude = OPERATING_ALTITUDE - ALTITUDE_INCREASE_ON_PATH_CONFLICT # increased altitude to avoid other drone!
    else:
        self.operating_altitude = OPERATING_ALTITUDE # normal altitude

    # make sure we are at the desired altitude
    if self.vehicle_local_position.z > self.operating_altitude + ALTITUDE_TOLERENCE:
        self.publish_position_setpoint_global(self.vehicle_global_position.lat, self.vehicle_global_position.lon, self.operating_altitude, OPERATING_VELOCITY)
        return True # signal that we are still increasing altitude

    return False # we are at the correct operating altitude now





def tsunami_online_loop(self):

    # # Wait until drone have reached initial operating altitude
    # if self.vehicle_local_position.z > OPERATING_ALTITUDE+ALTITUDE_TOLERENCE and not self.initial_alt_reached:  # (remember NED coordinates: down is positive) (0.5m tolerance)
    #     self.publish_position_setpoint_global(self.home_pos.lat, self.home_pos.lon, OPERATING_ALTITUDE, OPERATING_VELOCITY)
    #     return
    # else:
    #     self.initial_alt_reached = True

    #self.get_logger().info(f"WTFF: target {self.lat_target}, {self.lon_target}, altitude {self.operating_altitude}, current pos {self.vehicle_global_position.lat}, {self.vehicle_global_position.lon}, current alt {self.vehicle_local_position.z}, path_clear {self.path_clear}, at_path_conflict_alt {self.at_path_conflict_alt}, waiting_for_path_check {self.waiting_for_path_check}")

    # Check if all cells have been visited
    if len(self.visited_cells) >= self.bf_traversal_size:
        all_cells_visited(self)
        return

    # Make sure we are at the correct operating altitude before continuing (increased if needed to avoid path conflicts)
    if set_correct_operating_altitude(self):
        return # wait until we have reached the correct operating altitude

    # Check if we have reached the target cell - if so, update to next target cell
    dist_to_target = great_circle_distance(self.vehicle_global_position.lat, self.vehicle_global_position.lon, self.lat_target, self.lon_target) # in meters
    if dist_to_target < WAYPOINT_REACHED_TOLERANCE:
        self.get_logger().info(f"Reached cell: {self.current_target_cell} at lat: {self.lat_target}, lon: {self.lon_target}")
        update_target_cell(self) # Update to next target cell

        # We have a new waypoint target - therefore, trigger a new path check:
        self.trigger_path_check = True 


    if ENABLE_PATH_CONFLICT_CHECK:

        if self.trigger_path_check:
            self.trigger_path_check = False # reset flag
            self.check_all_current_paths() # will set self.path_clear (to something other than None) when it completes (also, self.at_path_conflict_alt will be set to signal if we need to increase altitude or not)
            self.waiting_for_path_check = True
        
        
        if self.waiting_for_path_check:
        # Question: why not just use path_clear instead of waiting_for_path_check?
        # Answer: we want self.path_clear to only be controlled by check_all_current_paths() and its callbacks - therefore, we introduce the waiting_for_path_check variable here, that we can reset whenever we want (i.e. here)
            if self.path_clear == None:
                return # wait for the path check to complete
            # we now got an answer from the path check!
            self.waiting_for_path_check = False 
                

    # Calculate yaw to target waypoint (if enabled)
    yaw_rad = 0.0
    if ENABLE_YAW_TURNING:
        yaw_rad = great_circle_bearing(self.vehicle_global_position.lat, self.vehicle_global_position.lon, self.lat_target, self.lon_target) # using the Great-circle Bearing Formula




    # ISSUE... når vi kommer op i increased alt, så skal den blive der, indtil vi har nået waypointet. 



    # ISSUE waiting_for_path_check skal sættes false, når vi har fået path_clear svar.. men det bliver den ikke mens vi venter på dronen flyver op til increased alt

    # ISSUE.. DEN SKAL HOLDE PÅ operating_altitude VÆRDIEN. NU FLYVER DEN OP OG SÅ NED MED DET SAMME IGEN MENS DEN FLYVER HEN TIL TARGET.


    # Publish position setpoint to target waypoint
    self.publish_position_setpoint_global(self.lat_target, self.lon_target, self.operating_altitude, OPERATING_VELOCITY, yaw_rad)

























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

