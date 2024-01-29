import numpy as np
# from utils import visualize_map
import picar_4wd as fc
# from picar_4wd import get_distance_at

# Constants
GRID_SIZE = 50
CAR_POS = (GRID_SIZE//2, 0)
ANGLE_INCREMENT = 10
MAX_DISTANCE = 0.8 * GRID_SIZE  # limit distance to filter out noise

# Initialize the grid
grid = np.zeros((GRID_SIZE, GRID_SIZE))


def polar_to_cartesian(angle, distance):
    """
    Convert polar coordinates (angle in degrees, distance) to Cartesian coordinates.
    Note that we consider angles in the 2nd to 1st quadrants
    Angles are measured wrt positive Y-axis (cw -> neg, ccw -> pos)
    """
    radian = np.radians(angle)
    x = -distance * np.sin(radian)
    y = distance * np.cos(radian)
    return int(x), int(y)


def update_grid(angle, distance, last_angle, last_distance):
    """
    Update the grid based on the sensor reading.
    """
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
        grid[x1, y1] = 1


def scan_environment():
    """
    Simulate scanning the environment and updating the grid.
    """
    last_angle = None
    last_distance = None
    #todo: read from last angle to avoid always turning to -90 first
    for angle in range(-90, 91, ANGLE_INCREMENT):
        # Ignore distances that are beyond our max range. This avoids unnecessary maneuvers based on distant objects
        if (distance := fc.get_distance_at(angle)) > MAX_DISTANCE:
            continue
        update_grid(angle, distance, last_angle, last_distance)

        last_angle = angle
        last_distance = distance

import random
# random.seed(88)
def read_ultrasonic_sensor(angle):
    """
    Placeholder function to simulate an ultrasonic sensor reading at a given angle.
    Replace this with the actual sensor reading logic.
    Simulated with random number between 1 and 100
    """
    # Example: return a fixed distance for testing
    return random.randint(1, GRID_SIZE)  # Replace with actual sensor reading


def draw_line(x0, y0, x1, y1):
    """
    Draw a line from (x0, y0) to (x1, y1) using Bresenham's line algorithm.
    Use Bresenham's algorithm for efficiency as it only involves integer arithmetic
    """
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy  # error value e_xy

    while True:
        if 0 <= x0 < GRID_SIZE and 0 <= y0 < GRID_SIZE:
            grid[x0, y0] = 1  # Set the point on the grid

        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:  # e_xy + e_x > 0
            err += dy
            x0 += sx
        if e2 <= dx:  # e_xy + e_y < 0
            err += dx
            y0 += sy




# Example usage
scan_environment()
np.set_printoptions(threshold=np.inf, linewidth=np.inf)
print(np.transpose(grid))
# visualize_map(grid)


