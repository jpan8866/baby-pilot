import numpy as np
# from utils import visualize_map
import matplotlib.pyplot as plt
import picar_4wd as fc
import settings

# Constants
GRID_SIZE = settings.GRID_SIZE
CAR_POS = (GRID_SIZE//2, 0)
ANGLE_INCREMENT = settings.ANGLE_INCREMENT
MAX_DISTANCE = 1 * GRID_SIZE  # limit distance to filter out noise
PADDING_SIZE = settings.PADDING_SIZE  # number of cells to pad around a point in grid
# Initialize the grid


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


def euclidean_distance(x1, y1, x2, y2):
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def update_grid(grid, angle, distance, last_angle, last_distance):
    """
    Update the grid based on the sensor reading.
    """
    x1, y1 = polar_to_cartesian(angle, distance)
    x1 += CAR_POS[0]
    y1 += CAR_POS[1]
    if 0 <= x1 < GRID_SIZE and 0 <= y1 < GRID_SIZE:
        add_padding(grid, x1, y1)
        if last_angle is not None:
            x0, y0 = polar_to_cartesian(last_angle, last_distance)
            x0 += CAR_POS[0]
            y0 += CAR_POS[1]
            if 0 <= x0 < GRID_SIZE and 0 <= y0 < GRID_SIZE and euclidean_distance(x0, y0, x1, y1) <= settings.MIN_DISTANCE_TO_INTERPOLATE:
                draw_line(grid, x0, y0, x1, y1)

def add_padding(grid, x, y):
    """
    Add padding of number of cells equal to padding_size in each direction around a given point (x, y) in the grid.
    """
    for i in range(-PADDING_SIZE, PADDING_SIZE+1):
        for j in range(-PADDING_SIZE, PADDING_SIZE+1):
            new_x = x + i
            new_y = y + j
            if 0 <= new_x < GRID_SIZE and 0 <= new_y < GRID_SIZE:
                grid[new_x, new_y] = 1


def scan_environment() -> [[]]:
    """
    This function calls update_grid to set grid cell values to 1.
    """
    last_angle = None
    last_distance = None
    grid = np.zeros((GRID_SIZE, GRID_SIZE))

    for angle in range(-90, 91, ANGLE_INCREMENT):
        # Ignore distances that are beyond our max range. This avoids unnecessary maneuvers based on distant objects
        for _ in range(2):
            distance = fc.get_distance_at(angle)
        if distance == -2:
            continue
        update_grid(grid, angle, distance, last_angle, last_distance)

        last_angle = angle
        last_distance = distance
    return grid


def draw_line(grid, x0, y0, x1, y1):
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
            add_padding(grid, x0, y0)  # Set the point on the grid

        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:  # e_xy + e_x > 0
            err += dy
            x0 += sx
        if e2 <= dx:  # e_xy + e_y < 0
            err += dx
            y0 += sy







# ------------------- Uncomment below for testing purposes ------------------
# below code moved to utils as well
# Example usage
# scan_environment()
# np.set_printoptions(threshold=np.inf, linewidth=np.inf)
# print(np.transpose(grid))
# visualize_map(grid)

if __name__ == "__main__":
    grid = scan_environment()
    grid[CAR_POS] = 3
    visual_grid = np.transpose(grid)
    # Display the transposed grid
    plt.figure(figsize=(8, 8))
    plt.title("Advanced Mapping")
    plt.imshow(visual_grid, cmap='gray', origin='lower')
    plt.colorbar()
    plt.show()
