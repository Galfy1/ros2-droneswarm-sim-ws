import numpy as np
import math

# Direction vectors
dRow_4way = [ -1, 0, 1, 0]
dCol_4way = [ 0, 1, 0, -1]

dRow_8way = [ -1, -1, 0, 1, 1, 1, 0, -1]
dCol_8way = [ 0, 1, 1, 1, 0, -1, -1, -1]

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



def _find_closest_cell(grid, current_cell, visited_cells):
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
    # MÅSKE LAV EN WARNING MED AT DET IKKE GIVER SUPER MEGET MENEING MED allow_diagonal_in_path = false for centroid stuff?

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



# "unidirectional" angle difference (0 to pi). "bidirectional" would be (0 to pi/2). 
# "bidirectional" does not seem to work very well for the "pure centroid" method (will sometimes do really sharp turns in direction)
def find_next_cell_centroid(grid, current_cell, visited_cells, centroid_line_angle: float, 
                             directional = "unidirectional", allow_diagonal_in_path = True, angle_offset_rad = 0):

    centroid_angle_diff_of_neighbors = _find_centroid_angle_diff_of_neighbors(grid, current_cell, visited_cells, centroid_line_angle+angle_offset_rad,
                                                                              directional, allow_diagonal_in_path)
    #print(f"centroid angle: {centroid_angle_diff_of_neighbors}")

    if centroid_angle_diff_of_neighbors: # if list is not empty
        # Find the neighbor with the smallest angle difference
        # centroid_angle_diff_of_neighbors is a list of (neighbor_cell, angle_diff_rad)
        neighbor_with_smallest_angle_diff = min(centroid_angle_diff_of_neighbors, key=lambda x: x[1])
        #print(f"Next cell chosen with angle diff: {neighbor_with_smallest_angle_diff[1]}")
        return neighbor_with_smallest_angle_diff[0] 
    else:
        # No unvisited neighbor is found. Find the closest unvisited cell
        #print("No unvisited neighbor found. Finding closest unvisited cell...")
        return _find_closest_cell(grid, current_cell, visited_cells) # closest unvisited cell. returns None if no more valid cells are left