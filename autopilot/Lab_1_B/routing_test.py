from path_finder import a_star_search, Node
from advanced_mapping import scan_environment
from utils import visualize_path, mark_path_on_grid
import numpy as np
from settings import GRID_SIZE

grid = scan_environment()

# Define the start and goal positions
start = Node(GRID_SIZE//2, 0)
goal = Node(GRID_SIZE//2, GRID_SIZE-1)

# Run A* search to find the path
path = a_star_search(grid, start, goal)
if path:
    mark_path_on_grid(grid, path)

np.savetxt('./grid.txt', grid, fmt='%d')  # scp file to laptop to view

visualize_path(grid)
