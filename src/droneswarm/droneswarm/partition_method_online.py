import numpy as np
import math
from .great_circle_tools import great_circle_distance, great_circle_bearing

OPERATING_ALTITUDE = -25.0  # meters (remeber, NED coordinates: down is positive)
OPERATING_VELOCITY = 1.0 # m/s # TODO DOES NOT CURRENTLY WORK IN THE PX4 SIM
ALTITUDE_TOLERENCE = 0.5  # meters, how close we need to be to the operating altitude to consider it "reached"
ENABLE_YAW_TURNING = True  # our real-world drone only has a 1D gimbal (pitch), so we want to turn the drone to face the direction of travel



def partition_method_online_init(self):
    self.operating_altitude = OPERATING_ALTITUDE # current operating altitude (can be increased to avoid path conflicts)
    self.current_target_cell = self.home_cell_from_offline # initialize to home cell
    self.lat_target, self.lon_target = cell_to_gps(self, self.current_target_cell) # get initial target gps coordinates
    self.flight_path_log = [(float(self.lat_target), float(self.lon_target))]  # list to log the flight path (lat, lon) tuples - for analysis later
    self.flight_complete = False 
    self.land_stabelize_counter = 0



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
    pass # TODO


# TODO 
# vi har nogle ting i px4_controller.py som skal slås fra hvis vi ikke laver tsunami
# 	f.eks. det komminukations halløj der..
# 		e.g. det depender på variabler der er lavet i tsunami_online.py.... (e.g. self.at_path_conflict_alt)… 

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