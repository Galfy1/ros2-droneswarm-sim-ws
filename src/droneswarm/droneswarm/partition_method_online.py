import numpy as np
import math
from .great_circle_tools import great_circle_distance, great_circle_bearing


OPERATING_ALTITUDE = -25.0  # meters (remeber, NED coordinates: down is positive)
OPERATING_VELOCITY = 1.0 # m/s # TODO DOES NOT CURRENTLY WORK IN THE PX4 SIM
ALTITUDE_TOLERENCE = 0.5  # meters, how close we need to be to the operating altitude to consider it "reached"
ENABLE_YAW_TURNING = True  # our real-world drone only has a 1D gimbal (pitch), so we want to turn the drone to face the direction of travel

WAYPOINT_REACHED_TOLERANCE = 1.0  # meters, how close we need to be to a waypoint to consider it "reached"


def partition_method_online_init(self):
    self.operating_altitude = OPERATING_ALTITUDE # current operating altitude (can be increased to avoid path conflicts)
    self.current_target_cell = self.home_cell_from_offline # initialize to home cell
    self.lat_target, self.lon_target = cell_to_gps(self, self.current_target_cell) # get initial target gps coordinates
    self.flight_path_log = [(float(self.lat_target), float(self.lon_target))]  # list to log the flight path (lat, lon) tuples - for analysis later
    self.flight_complete = False 
    self.land_stabelize_counter = 0
    self.cell_progress_index = 0  # index to track progress through the path cells

def cell_to_gps(self, cell):
    return self.fly_nofly_grid_gps[cell[0]][cell[1]]

def all_cells_visited(self):
    #self.get_logger().info("Completed all cells.")

    # Print the flight path log
    if self.flight_complete:
        return # already completed


    # TODO der er ingen path collision chechs mens vi returner hjem...

    # Return to home position and land:
    #self.get_logger().info("Returning to home position and landing.")
    # Calculate yaw to target waypoint (if enabled)

    # TODO hvorfor gør jeg det her: ?? hvis jeg aligvel bare bruger home pos direkte nedenunder??
    self.lat_target = self.home_pos.lat 
    self.lon_target = self.home_pos.lon

    yaw_rad = 0.0
    if ENABLE_YAW_TURNING:
        yaw_rad = great_circle_bearing(self.vehicle_global_position.lat, self.vehicle_global_position.lon, self.lat_target, self.lon_target)

    self.publish_position_setpoint_global(self.home_pos.lat, self.home_pos.lon, OPERATING_ALTITUDE, OPERATING_VELOCITY, yaw_rad) # go to home position at ground level

    # Check if we have reached home position:
    dist_to_home = great_circle_distance(self.vehicle_global_position.lat, self.vehicle_global_position.lon, self.home_pos.lat, self.home_pos.lon) # in meters
    if dist_to_home < 0.3:  # within 30cm of home position

        # wait a bit to stabilize before landing:
        if self.land_stabelize_counter < self.one_sec_loop_count*2:
            self.land_stabelize_counter += 1
            return

        self.get_logger().info("Reached home position, landing now.")
        self.land() 
        self.get_logger().info(f"FLIGHT PATH LOG: {self.flight_path_log}")
        self.flight_complete = True 



def set_correct_operating_altitude(self):
        # make sure we are at the desired altitude
    if self.vehicle_local_position.z > OPERATING_ALTITUDE + ALTITUDE_TOLERENCE:
        self.publish_position_setpoint_global(self.vehicle_global_position.lat, self.vehicle_global_position.lon, OPERATING_ALTITUDE, OPERATING_VELOCITY)
        return True # signal that we are still increasing altitude

    return False # we are at the correct operating altitude now

def update_target_cell(self):

    # make sure we dont go out of bounds:
    if self.cell_progress_index >= self.path_size:
        self.get_logger().warning("All cells have been visited.")
        all_cells_visited(self)
        return

    next_cell = self.uav_path[self.cell_progress_index]

    self.current_target_cell = next_cell
    self.lat_target, self.lon_target = cell_to_gps(self, next_cell)
    self.visited_cells.add(next_cell)
    self.flight_path_log.append((float(self.lat_target), float(self.lon_target)))

    self.cell_progress_index += 1


def partition_method_online_loop(self):
    
    # Check if all cells have been visited
    if len(self.visited_cells) >= self.path_size:
        all_cells_visited(self)
        return

    # Make sure we are at the correct operating altitude before continuing
    if set_correct_operating_altitude(self):
        return # wait until we have reached the correct operating altitude

    # Check if we have reached the target cell - if so, update to next target cell
    dist_to_target = great_circle_distance(self.vehicle_global_position.lat, self.vehicle_global_position.lon, self.lat_target, self.lon_target) # in meters
    if dist_to_target < WAYPOINT_REACHED_TOLERANCE:
        self.get_logger().info(f"Reached cell: {self.current_target_cell} at lat: {self.lat_target}, lon: {self.lon_target}")
        update_target_cell(self) # Update to next target cell


    # Calculate yaw to target waypoint (if enabled)
    yaw_rad = 0.0
    if ENABLE_YAW_TURNING:
        yaw_rad = great_circle_bearing(self.vehicle_global_position.lat, self.vehicle_global_position.lon, self.lat_target, self.lon_target) # using the Great-circle Bearing Formula


    # Publish position setpoint to target waypoint
    self.publish_position_setpoint_global(self.lat_target, self.lon_target, OPERATING_ALTITUDE, OPERATING_VELOCITY, yaw_rad)