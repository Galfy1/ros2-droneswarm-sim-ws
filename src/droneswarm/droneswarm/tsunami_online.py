#from haversine import haversine, Unit
import numpy as np
import math
from .tsunami_find_next_cell import find_next_cell_bft, find_next_cell_centroid, find_next_cell_hybrid
from .great_circle_tools import great_circle_distance, great_circle_bearing



# ADJUSTABLE PARAMETERS:

PATH_PLANNING_METHOD = 'centroid180'  # Options: "BFT", "centroid90", "centroid180", "centroid_hybrid", "centroid90_hybrid"
ALLOW_DIAGONAL_PATH_PLANNING = True  # if True, diagonal neighbors are considered neighbors when finding the next cell to visit. If False, only N/S/E/W neighbors are considered.
HYBRID_CENTROID_WEIGHT = 0.6  # only used if PATH_PLANNING_METHOD is hybrid - weight for centroid direction in hybrid path planning (0.0 = only current direction, 1.0 = only centroid direction)

OPERATING_ALTITUDE = -25.0  # meters (remeber, NED coordinates: down is positive)
OPERATING_VELOCITY = 1.0 # m/s # TODO DOES NOT CURRENTLY WORK IN THE PX4 SIM
ALTITUDE_TOLERENCE = 0.5  # meters, how close we need to be to the operating altitude to consider it "reached"
ENABLE_YAW_TURNING = True  # our real-world drone only has a 1D gimbal (pitch), so we want to turn the drone to face the direction of travel

ENABLE_PATH_CONFLICT_CHECK = True  # if True, the drone will check if its path conflicts with other drones and adjust altitude if needed
WAYPOINT_REACHED_TOLERANCE = 1.0  # meters, how close we need to be to a waypoint to consider it "reached"

ALTITUDE_INCREASE_ON_PATH_CONFLICT = 5.0 # meters, how much to increase altitude if a path conflict is detected with another drone








def tsunami_online_init(self):
    #self.initial_alt_reached = False
    self.current_target_cell = self.home_cell_from_offline # initialize to home cell
    self.lat_target, self.lon_target = cell_to_gps(self, self.current_target_cell) # get initial target gps coordinates
    self.flight_path_log = [(float(self.lat_target), float(self.lon_target))]  # list to log the flight path (lat, lon) tuples - for analysis later
    self.flight_complete = False 
    self.trigger_path_check = False
    self.waiting_for_path_check = False 
    self.land_stabelize_counter = 0
    self.operating_altitude = OPERATING_ALTITUDE # current operating altitude (can be increased to avoid path conflicts)

    # For "hybrid" method, we also need the current direction angle
    self.current_direction_angle = 0.0  # initial direction angle (radians). Could be set to any value, as it will be updated after the first move.

    self.get_logger().info("Tsunami online initialized")


def all_cells_visited(self):
    #self.get_logger().info("Completed all cells.")

    # Print the flight path log
    if self.flight_complete:
        return # already completed

    # TODO there are no path collision checks while returning home... this needs to be fixed if used in real-world scenarios.

    # Return to home position and land:
    # Calculate yaw to target waypoint (if enabled)

    # TODO not sure the following two lines are needed... we use the home pos directly anyway
    self.lat_target = self.home_pos.lat 
    self.lon_target = self.home_pos.lon

    yaw_rad = 0.0
    if ENABLE_YAW_TURNING:
        yaw_rad = great_circle_bearing(self.vehicle_global_position.lat, self.vehicle_global_position.lon, self.lat_target, self.lon_target)

    self.publish_position_setpoint_global(self.home_pos.lat, self.home_pos.lon, self.operating_altitude, OPERATING_VELOCITY, yaw_rad) # go to home position at ground level

    # Check if we have reached home position:
    dist_to_home = great_circle_distance(self.vehicle_global_position.lat, self.vehicle_global_position.lon, self.home_pos.lat, self.home_pos.lon) # in meters
    if dist_to_home < 0.5:  # within 50cm of home position

        # wait a bit to stabilize before landing:
        if self.land_stabelize_counter < self.one_sec_loop_count*2:
            self.land_stabelize_counter += 1
            return

        self.get_logger().info("Reached home position, landing now.")
        self.land() 

        # Log sim results: 
        self.mission_end_time_log = int(self.get_clock().now().nanoseconds / 1000) # microseconds
        self.flight_path_log.append((self.home_pos.lat, self.home_pos.lon))  # also add the flight back to home to the log (we need to to later calcute the fill path length)
        self.get_logger().info(f"FLIGHT PATH LOG: {self.flight_path_log}")
        self.get_logger().info(f"SIM START TIME (us): {self.sim_start_time_log}")
        self.get_logger().info(f"MISSION START TIME (us): {self.mission_start_time_log}")
        self.get_logger().info(f"MISSION END TIME (us): {self.mission_end_time_log}")

        self.flight_complete = True 



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





def update_target_cell(self):

    # Find the next cell in the Breadth First Traversal
    if PATH_PLANNING_METHOD == 'BFT':
        next_cell = find_next_cell_bft(self, self.bf_traversal_cells, self.current_target_cell, self.visited_cells, ALLOW_DIAGONAL_PATH_PLANNING)
    elif PATH_PLANNING_METHOD == 'centroid90':
        next_cell = find_next_cell_centroid(self, self.fly_nofly_grid, self.current_target_cell, self.visited_cells,
                                            self.centroid_line_angle, allow_diagonal_in_path=ALLOW_DIAGONAL_PATH_PLANNING, angle_offset_rad=math.pi/2)
    elif PATH_PLANNING_METHOD == 'centroid180':
        next_cell = find_next_cell_centroid(self, self.fly_nofly_grid, self.current_target_cell, self.visited_cells,
                                            self.centroid_line_angle, allow_diagonal_in_path=ALLOW_DIAGONAL_PATH_PLANNING, angle_offset_rad=math.pi)
    elif PATH_PLANNING_METHOD == 'centroid_hybrid':
        current_cell = self.current_target_cell
        next_cell = find_next_cell_hybrid(self, self.fly_nofly_grid, self.current_target_cell, self.visited_cells,
                                         self.centroid_line_angle, self.current_direction_angle,
                                         weight_centroid=HYBRID_CENTROID_WEIGHT,
                                         allow_diagonal_in_path=ALLOW_DIAGONAL_PATH_PLANNING)
        # Update current direction angle
        if next_cell is not None:
            self.current_direction_angle = math.atan2(next_cell[0] - current_cell[0], next_cell[1] - current_cell[1]) 
    elif PATH_PLANNING_METHOD == 'centroid90_hybrid':
        current_cell = self.current_target_cell
        next_cell = find_next_cell_hybrid(self, self.fly_nofly_grid, self.current_target_cell, self.visited_cells,
                                         self.centroid_line_angle, self.current_direction_angle,
                                         weight_centroid=HYBRID_CENTROID_WEIGHT,
                                         allow_diagonal_in_path=ALLOW_DIAGONAL_PATH_PLANNING, angle_offset_rad=math.pi/2)
        # Update current direction angle
        if next_cell is not None:
            self.current_direction_angle = math.atan2(next_cell[0] - current_cell[0], next_cell[1] - current_cell[1])                

    if next_cell is not None:
        # New target cell found!
        self.current_target_cell = next_cell
        self.lat_target, self.lon_target = cell_to_gps(self, next_cell)
        # Mark the next cell as "visited" (both locally and broadcast to other drones) - this is to "reserve" the cell for this drone
        self.visited_cells.add(next_cell)
        self.broadcast_visited_cell(next_cell) 
        self.flight_path_log.append((float(self.lat_target), float(self.lon_target))) # log the flight path for later plotting
    else:
        # self.get_logger().info("No valid next cell found.")
        # this means all cells have been visited
        all_cells_visited(self) 


def cell_to_gps(self, cell):
    return self.fly_nofly_grid_gps[cell[0]][cell[1]]




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

    # Publish position setpoint to target waypoint
    self.publish_position_setpoint_global(self.lat_target, self.lon_target, self.operating_altitude, OPERATING_VELOCITY, yaw_rad)
