from path_finder import a_star_search_4dir, Node
from advanced_mapping import scan_environment
from utils import visualize_path, visualize_map

grid = scan_environment()
# Create a 50x50 grid where 0 represents empty cells
GRID_SIZE = 50

# Define the start and goal positions
start = Node(GRID_SIZE//2, 0)
goal = Node(GRID_SIZE//2, GRID_SIZE-1)

# Run A* search to find the path
path = a_star_search_4dir(grid, start, goal)
visualize_path(grid, path)

# if path:
#     print("Path found:")
#     for x, y in path:
#         print(f"({x}, {y})")
# else:
#     print("No path found.")