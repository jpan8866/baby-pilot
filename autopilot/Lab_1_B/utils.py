import cv2
import numpy as np
import matplotlib.pyplot as plt


def visualize_map(grid, scale=10):
    """
    Visualize a 2D grid map using OpenCV.

    Args:
    - grid: 2D numpy array representing the map.
    - scale: Factor to scale up the image for better visualization.
    """
    grid = np.transpose(grid)
    # Scale up the grid for better visualization
    large_grid = cv2.resize(grid, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    # Convert the grid to an 8-bit unsigned integer type
    large_grid = large_grid.astype(np.uint8)
    # Convert to a color image
    color_map = cv2.cvtColor(large_grid, cv2.COLOR_GRAY2BGR)
    # Create a boolean mask where the obstacle cells (value 1) are located
    obstacle_mask = large_grid == 1
    # Apply the red color to the obstacle positions in the color_map
    color_map[obstacle_mask] = [0, 0, 255]  # Red color for obstacles

    # Show the map
    cv2.imshow("Mapping visualization", color_map)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def visualize_path(grid):
    np.set_printoptions(threshold=np.inf, linewidth=np.inf)
    # print(np.transpose(grid))

    # uncomment below if you have monitor plugged in to pi
    # Display the transposed grid
    visual_grid = np.transpose(grid)
    plt.figure(figsize=(8, 8))
    plt.title("Routing")
    plt.imshow(visual_grid, cmap='gray', origin='lower')
    plt.colorbar()
    plt.show()


def mark_path_on_grid(grid, path, path_value=3):
    for x, y in path:
        grid[x, y] = path_value


def find_edge_point(start, goal, GRID_SIZE):
    x1, y1 = start
    x2, y2 = goal
    edge_point = None

    # Handle vertical line case
    if x1 == x2:
        if y2 > y1:
            edge_point = (x1, GRID_SIZE - 1)
        else:
            edge_point = (x1, 0)
        return edge_point

    # Calculate slope
    m = (y2 - y1) / (x2 - x1)
    # Calculate y-intercept
    b = y1 - m * x1

    # Find potential intersections with grid boundaries
    intersections = []
    # Top edge
    intersections.append(((GRID_SIZE - 1 - b) / m, GRID_SIZE - 1))
    # Bottom edge
    intersections.append((-b / m, 0))
    # Right edge
    intersections.append((GRID_SIZE - 1, m * (GRID_SIZE - 1) + b))
    # Left edge
    intersections.append((0, b))

    # Filter valid intersections
    valid_intersections = [p for p in intersections if 0 <= p[0] < GRID_SIZE and 0 <= p[1] < GRID_SIZE]

    # Select the intersection that is in the direction of the goal
    if valid_intersections:
        edge_point = min(valid_intersections, key=lambda p: ((p[0] - x1) ** 2 + (p[1] - y1) ** 2))

    return edge_point

