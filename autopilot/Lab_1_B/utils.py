import cv2
import numpy as np

def visualize_map(grid, scale=10):
    """
    Visualize a 2D grid map using OpenCV.

    Args:
    - grid: 2D numpy array representing the map.
    - scale: Factor to scale up the image for better visualization.
    """
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