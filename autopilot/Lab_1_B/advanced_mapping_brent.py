import heapq
import math
import time
import numpy as np
import matplotlib.pyplot as plt
import picar_4wd as fc
import argparse
from advanced_mapping import scan_environment as scan_env

# Constants
GRID_SIZE = 50 # creating a 50x50 2d grid
# CAR_POS = (0, GRID_SIZE//2) # car position at bottom center of grid;
CAR_POS = (GRID_SIZE//2, 0) # car position at bottom center of grid; 
ANGLE_INCREMENT = 10
MAX_DISTANCE = 0.8 * GRID_SIZE  # limit distance to filter out noise

# Initialize the grid
current_grid: np.ndarray = np.zeros((GRID_SIZE, GRID_SIZE)) # the grid is zeroed by default and the ultrasonic sensor indicates where the 1's should be

# parge 2 numbers from the command line
parser = argparse.ArgumentParser()
parser.add_argument('-d', '--destination', nargs=2, metavar=('delta_x_cm', 'delta_y_cm'), type=int, help="x and y distances in cm to the destination, relative to the car")
args = parser.parse_args()
goal_delta_x, goal_delta_y = args.destination


####################### Sensing and Mapping Functions ############################
def destination_to_coordinates(delta_x: int, delta_y: int) -> tuple[int]:
    """
    Convert a relative destination in cm to coordinates on the grid.
    Coordinates are relative to the car's position.
    """
    # hope they don't give negative values for delta_y bc we're not moving backwards lol
    return (CAR_POS[0] + delta_x, CAR_POS[1] + abs(delta_y)) 

def polar_to_cartesian(angle: int, distance: int) -> tuple[int]:
# def polar_to_cartesian(angle, distance):
    """
    Convert polar coordinates (angle in degrees, distance) to Cartesian coordinates.
    Note that we consider angles in the 2nd to 1st quadrants
    Angles are measured wrt positive Y-axis (cw -> neg, ccw -> pos)
    This function returns an (x,y) location on the grid.
    """
    radian = np.radians(angle)
    x = -distance * np.sin(radian)
    y = distance * np.cos(radian)
    return int(x), int(y)

def update_grid(angle: int, distance: int, last_angle: int or None, last_distance: int or None) -> None:
    """
    Update the grid based on the sensor reading.
    This function manually sets appropriate cells to 1 in the grid,
    and using the draw_line function when appropriate.
    """
    global current_grid

    x1, y1 = polar_to_cartesian(angle, distance)
    x1 += CAR_POS[0]
    y1 += CAR_POS[1]
    print(x1, y1, distance, angle, last_angle, last_distance)

    if last_angle is not None and abs(last_angle - angle) == ANGLE_INCREMENT:
        # Interpolate between the last point and the current point if the delta is 1 angle increment
        x0, y0 = polar_to_cartesian(last_angle, last_distance)
        x0 += CAR_POS[0]
        y0 += CAR_POS[1]

        draw_line(x0, y0, x1, y1)
    elif 0 <= x1 < GRID_SIZE and 0 <= y1 < GRID_SIZE:
        current_grid[x1, y1] = 1

def scan_environment() -> None:
    """
    Simulate scanning the environment and updating the grid.
    This function calls update_grid to set grid cell values to 1.
    """
    last_angle = None
    last_distance = None
    #todo: read from last angle to avoid always turning to -90 first
    for angle in range(-90, 91, ANGLE_INCREMENT):
        # Ignore distances that are beyond our max range. This avoids unnecessary maneuvers based on distant objects
        if not 0 <= (distance := fc.get_distance_at(angle)) <= MAX_DISTANCE:
            continue
        update_grid(angle, distance, last_angle, last_distance)

        last_angle = angle
        last_distance = distance

def draw_line(x0: int, y0: int, x1: int, y1: int) -> None:
    """
    Draw a line from (x0, y0) to (x1, y1) using Bresenham's line algorithm.
    Use Bresenham's algorithm for efficiency as it only involves integer arithmetic
    This function sets the appropriate cells to 1 on the grid.
    """
    global current_grid

    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy  # error value e_xy

    while True:
        if 0 <= x0 < GRID_SIZE and 0 <= y0 < GRID_SIZE:
            current_grid[x0, y0] = 1  # Set the point on the grid

        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:  # e_xy + e_x > 0
            err += dy
            x0 += sx
        if e2 <= dx:  # e_xy + e_y < 0
            err += dx
            y0 += sy

####################### Routing Functions ############################
class Node:
    # A node class for A* Pathfinding
    def __init__(self, position, obstacle):
        self.position = position
        self.obstacle = obstacle
        self.g = float('inf')  # Initial cost from start to this node
        self.h = 0  # Heuristic estimate from this node to goal
        self.f = float('inf')  # Total cost (g + h)
        self.parent = None  # Keep track of the path
        
    def __lt__(self, other):
        return self.f < other.f
    def __gt__(self, other):
        return self.f > other.f
    def __eq__(self, other):
        return self.position == other.position
    def __ne__(self, other):
        return not self.__eq__(other)

def get_cost(current_node, neighbor) -> int:
    # Get the cost of moving from the current node to the neighbor
    # Returns the cost based on the direction of movement
    # The cost is 1 for vertical movement but 2 for horizontal or diagonal movement because the car has to turn and then move
    # if current_node.position[0] == neighbor.position[0] or current_node.position[1] == neighbor.position[1]:
    #     return 1
    # else:
    #     return 1.8
    return 1

def get_heuristic(node, goal) -> int:
    # Get the heuristic value for the neighbor
    # A simple Euclidean distance heuristic that returns integers
    # return int(np.sqrt((node.position[0] - goal.position[0])**2 + (node.position[1] - goal.position[1])**2)) # Euclidean distance
    return abs(node.position[0] - goal.position[0]) + abs(node.position[1] - goal.position[1]) # Manhattan distance

def calculate_straight_line_path(start, end):
    # Calculate a straight-line path from start to end
    start = np.array(start)
    end = np.array(end)

    # Calculate the direction vector
    direction = end - start

    # Calculate the distance between start and end points
    distance = np.linalg.norm(direction)

    # Normalize the direction vector
    normalized_direction = direction / distance

    # Calculate the number of points for the path (adjust as needed)
    num_points = int(distance) + 1

    # Generate the straight-line path
    straight_line_path = [tuple(np.round(start + i * normalized_direction).astype(int)) for i in range(num_points)]

    # print the straight line path
    print("*** straight line path ***: ", straight_line_path)

    return straight_line_path

def astar(grid, start, goal) -> list[tuple[int]] or None:
    # Implementation of A* algorithm

    def get_neighbors(current_node):
        # Get the neighbors of the current node
        # Returns a list of neighboring nodes
        neighbors = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                if i == 0 and j == 0:
                    continue
                neighbor_position = (current_node.position[0] + i, current_node.position[1] + j)
                if 0 <= neighbor_position[0] < len(grid) and 0 <= neighbor_position[1] < len(grid[0]):
                    neighbor = Node(neighbor_position, grid[neighbor_position[0]][neighbor_position[1]])
                    neighbors.append(neighbor)
        return neighbors

    start_node = Node(start, 0) # initialize position and 0 obstacle bool value
    start_node.g = 0
    print("start node: ", start_node.position, start_node.obstacle, start_node.g)
    goal_node = Node(goal, 0) # initialize position and 0 obstacle bool value
    print("goal node: ", goal_node.position, goal_node.obstacle)
    if start_node == goal_node:
        print("*** completed path ***")
        return [start_node.position]

    open_set = []
    heapq.heappush(open_set, (start_node.f, start_node))
    closed_set = set()

    while open_set:
        current_node = heapq.heappop(open_set)[1]
        print("****** current node: ", current_node.position, current_node.obstacle, current_node.g, current_node.h, current_node.f)

        if current_node.position == goal_node.position:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1] # Return the path in reverse
        

        closed_set.add(current_node.position)

        # print("number of neighbors: ", len(get_neighbors(current_node)))
        for neighbor in get_neighbors(current_node):
            # print("neighbor: ", neighbor.position, neighbor.obstacle, neighbor.g)
            if neighbor.position in closed_set or neighbor.obstacle == 1:
                continue

            tentative_g = current_node.g + get_cost(current_node, neighbor)
            # print("tentative_g: ", tentative_g)

            if tentative_g < neighbor.g:
                neighbor.g = tentative_g
                neighbor.h = get_heuristic(neighbor, goal_node)
                neighbor.f = neighbor.g + neighbor.h
                # print("neighbor.f: ", neighbor.f)
                neighbor.parent = current_node
                heapq.heappush(open_set, (neighbor.f, neighbor))

    return None  # No path found

####################### Control Functions ############################
def follow_path(path, sleep_factor=0.05, power=10) -> list[int]:
    # Follow the path using the car
    # Returns a list of angles turned
    print("following path....")
    # Initialize a list to store the angles
    angles = []
    i = 0
    try:
        while i < len(path) - 1:
            current_point = path[i]
            print("current point: ", current_point)
            next_point = path[i + 1]
            print("next point: ", next_point)

            # Calculate the angle between the current point and the next point
            angle = calculate_angle(current_point, next_point)
            angles.append(angle)
            print("turn angle: ", angle)

            # Count consecutive points in the same direction
            consecutive_points = 0
            while angle == calculate_angle(current_point, path[i + 1]) and i < len(path) - 2:
                i += 1
                next_point = path[i + 1]
                consecutive_points += 1
            print("consecutive points same direction: ", consecutive_points)

            # Turn the car to the calculated angle if needed
            if angle > 0:
                fc.turn_right(power)  # Adjust the power as needed
                time.sleep(abs(angle) * 0.01 * 0.8)
            elif angle < 0:
                fc.turn_left(power)  # Adjust the power as needed
                time.sleep(abs(angle) * 0.01 * 0.8)
            print("turned.... if needed")
            print("moving forward...")
            

            # Move the car forward to the next point
            fc.forward(power)  # Adjust the power as needed

            # Sleep for a factor of the number of consecutive points skipped
            time.sleep(consecutive_points * sleep_factor) if consecutive_points > 0 else time.sleep(sleep_factor)

            # Stop the car
            fc.stop()
            print("stopped. will change directions...")
            time.sleep(sleep_factor)
            

            i += 1
        
    finally:
        fc.stop()
        return angles

    # Stop the car when the path is completed
#     fc.stop()

def calculate_angle(current_point, next_point) -> float:
    # Calculate the angle between the current point and the next point
    delta_x = next_point[0] - current_point[0]
    delta_y = next_point[1] - current_point[1]
    angle_radians = math.atan2(delta_y, delta_x)
    angle_degrees = math.degrees(angle_radians)
    # Convert the angle to the range (-180, 180]
    if delta_x == 0:
        angle_degrees -= 90
    elif delta_x < 0:
        angle_degrees = angle_degrees * -1 + 90 # Convert to negative angle
    elif delta_x > 0:
        if delta_y < 0:
            angle_degrees = angle_degrees * -1 + 360
        else:
            angle_degrees = 90 - angle_degrees

    return angle_degrees
    
####################### Visualization Helper ############################
def visualize_grid(grid, path, start, goal):
    plt.figure(figsize=(8, 8))
    plt.title("Grid Visualization")

    # Display the grid and transpose it
    plt.imshow(np.transpose(grid), cmap='Greys', origin='lower')

    # Plot obstacles in red and transpose it to match the grid
    obstacles_transposed = np.argwhere(grid == 1)
    plt.scatter(obstacles_transposed[:, 0], obstacles_transposed[:, 1], color='red', marker='x', s=100)
    # obstacles = np.argwhere(grid == 1)
    # plt.scatter(obstacles[:, 1], obstacles[:, 0], color='red', marker='x', s=100)

    # Plot start
    plt.scatter(start[0], start[1], color='green', marker='o', s=100, label='Start')

    # Plot goal
    plt.scatter(goal[0], goal[1], color='red', marker='o', s=100, label='Goal')

    # Plot the path
    if path:
        path = np.array(path)
        plt.plot(path[:, 0], path[:, 1], color='blue', linewidth=2, label='Path')

    plt.legend()
    plt.show()

# Example usage
# scan_environment()
# np.set_printoptions(threshold=np.inf, linewidth=np.inf)
# print(np.transpose(grid))
# # visualize_map(grid)

# grid[CAR_POS] = 2 # this sets the color of the car to white while objects are gray
# visual_grid = np.transpose(grid)
# # Display the transposed grid
# # plt.figure(figsize=(8, 8))
# fig, ax = plt.subplots()
# plt.title("Mapping")
# fig1 = ax.imshow(visual_grid, cmap='gray', origin='lower')
# fig2 = ax.imshow(grid, cmap='inferno', origin='lower', alpha=0.5)
# # plt.plot(grid[:,0], color='r')
# # plt.imshow(grid, cmap='gray', origin='lower')
# fig.colorbar(fig1, ax=ax, fraction=0.046, pad=0.04)
# fig.colorbar(fig2, ax=ax, fraction=0.046, pad=0.08)
# # plt.imshow(visual_grid, cmap='gray', origin='lower')
# # plt.colorbar()
# plt.show()

#######################################################
def route_once():
    global current_grid
    scan_environment()
    np.set_printoptions(threshold=np.inf, linewidth=np.inf)

    current_grid[CAR_POS] = 2 # this sets the color of the car to white while objects are gray
    visual_grid = np.transpose(current_grid)
    # Display the transposed grid
    plt.figure(figsize=(8, 8))
    plt.title("Mapping")
    plt.imshow(visual_grid, cmap='gray', origin='lower')
    plt.colorbar()
    plt.show()

    # Define the start and goal
    start = CAR_POS
    goal = (GRID_SIZE-1, GRID_SIZE-1) # TODO make this an argument to pass to the function

    # Run A* algorithm
    path = astar(current_grid, start, goal)
    print(path)

    # Visualize the grid
    visualize_grid(current_grid, path, start, goal)

def route_continuously():
    """
    Continuously route to the destination.
    When the destination is outside the range of the local grid, this function 
    takes the approach of making a path from destination to start, and subtracting
    the the distance needed until it's within the range of the local grid.
    """
    global current_grid
    global CAR_POS
    global GRID_SIZE

    global_target = destination_to_coordinates(goal_delta_x, goal_delta_y)
    
    while True:
        # scan the environment and update the current grid
        print("scanning the environment...")
        scan_environment()

        # check if the target is within the range of the current local grid
        # if it is, then run the A* algorithm and follow the path
        if (0 <= global_target[0] <= GRID_SIZE
        and 0 <= global_target[1] < GRID_SIZE):
            print("Condition 1: global target within range of local grid")
            # Run A* algorithm
            path = astar(current_grid, CAR_POS, global_target)
            print(path)

            # Visualize the grid
            # visualize_grid(current_grid, path, CAR_POS, global_target)

            # Follow the path
            follow_path(path)
            print("!!!!!!!!! Path successfully followed !!!!!!!!!")
            break # stop the loop after the path is followed

        # check if the target is initially beyond the range of the current local grid
        # if it is, then move the car to the edge of the current grid and update the local grid
        elif (global_target[0] >= GRID_SIZE or global_target[0] <= 0
        or global_target[1] >= GRID_SIZE):
            print("Condition 2: global target beyond range of local grid")
            # calculate a path to the edge of the current local grid in the direction of the global target
            straight_line_path = calculate_straight_line_path(CAR_POS, global_target)

            # find point on the straight line path that is on the edge of the local grid
            local_target = next((point for point in straight_line_path if point[0] <= 0 or point[0] >= GRID_SIZE or point[1] >= GRID_SIZE), None)
            print("local target: ", local_target)
            edge_path = astar(current_grid, CAR_POS, local_target)

            # Visualize the grid
            visualize_grid(current_grid, edge_path, CAR_POS, local_target)

            # Follow the path
            turn_angles = follow_path(edge_path)
            print("!!!!!!!!! Car has reached the edge of the local grid !!!!!!!!!")

            # CAR_POS is still at (25, 0) so adjust the frame of reference
            # the new global target is the old global target minus the distance to the edge of the local grid
            # emphasis on distance to the local grid, not just the local target since the car's position is not (0, 0)
            global_target = (global_target[0] - (local_target[0] - CAR_POS[0]), global_target[1] - (local_target[1] - CAR_POS[1]))
            print("new global target: ", global_target)

            # update the local grid
            current_grid = np.zeros((GRID_SIZE, GRID_SIZE))

            # reset the car direction to 0 degrees
            print("resetting car direction to 0 degrees")
            print("turned angles: ", turn_angles)
            try:
                for angle in reversed(turn_angles):
                    if angle < 0:
                        fc.turn_right(10)  # Adjust the power as needed
                        time.sleep(abs(angle) * 0.01 * 0.8)
                    elif angle > 0:
                        fc.turn_left(10)  # Adjust the power as needed
                        time.sleep(abs(angle) * 0.01 * 0.8)
            finally:
                fc.stop()
                print("car direction reset to 0 degrees... waiting before scanning the environment again")
                time.sleep(0.5)
        else:
            print("Condition 3: global target within range of local grid but the path is not found... exiting loop")
            print("global target: ", global_target)
            break


        # wait for a few seconds before scanning the environment again
        time.sleep(1)
            
        
# route_once()



####################### Testing ############################
def generate_mock_grid(rows: int, cols: int, obstacles=[]) -> np.ndarray:
    # Initialize grid with all zeros (no obstacles)
    grid = np.zeros((rows, cols), dtype=int)

    # Place obstacles in the specified positions
    for obstacle in obstacles:
        row, col = obstacle
        grid[row, col] = 1  # Set the obstacle value to 1

    return grid

def mock_route_once():
    # Place obstacles at specific positions
    # obstacle_positions = [(22, 10), (23, 10), (24, 10), (25, 10), (26, 10), (27, 10), (28, 10), (29, 10), (30, 10), (31, 10), (32, 10), (33, 10), (34, 10), (35, 10), (36, 10), (37, 10), (38, 10), (39, 10), (40, 10), (41, 10), (42, 10), (43, 10), (44, 10), (45, 10), (46, 10), (47, 10), (48, 10), (49, 10), (15, 20), (16, 20), (17, 20), (18, 20), (19, 20), (20, 20), (21, 20), (22, 20), (23, 20), (24, 20), (25, 20), (26, 20), (27, 20), (28, 20), (29, 20), (30, 20)]
    obstacle_positions = []

    # Generate mock grid
    mock_grid = generate_mock_grid(50, 50, obstacles=obstacle_positions)
    print("Mock grid: ", mock_grid)

    # Define the start and goal
    start = CAR_POS
    goal = destination_to_coordinates(goal_delta_x, goal_delta_y) # 10cm to the right and 25cm ahead

    # Run A* algorithm
    # TODO return error if goal is on edge of grid and thus unreachable
    path = astar(mock_grid, start, goal) # TODO factor in a boundary buffer for obstacles
    print("Path: ", path)
    print("Path length: ", len(path))


    # Visualize the grid
    visualize_grid(mock_grid, path, start, goal)

    # Follow the path
    follow_path(path)


def route():
    
    scanned_grid = scan_env()
    np.set_printoptions(threshold=np.inf, linewidth=np.inf)

    # Define the start and goal
    
    start = CAR_POS
    goal = (25, 49)

    # Run A* algorithm
    path = astar(scanned_grid, start, goal) # TODO factor in a boundary buffer for obstacles
    print("Path: ", path)
    print("Path length: ", len(path))


    # Visualize the grid
    # visualize_grid(scanned_grid, path, start, goal)

    # Follow the path
    follow_path(path)
    
# mock_route_once()
route_continuously()
