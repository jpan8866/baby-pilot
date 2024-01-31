from path_finder import a_star_search, Node
from advanced_mapping import scan_environment

grid = scan_environment()

# Create a 50x50 grid where 0 represents empty cells
GRID_SIZE = 50

# Define the start and goal positions
start = Node(0, 0)
goal = Node(GRID_SIZE - 1, GRID_SIZE - 1)

# Run A* search to find the path
path = a_star_search(grid, start, goal)

if path:
    print("Path found:")
    for x, y in path:
        print(f"({x}, {y})")
else:
    print("No path found.")