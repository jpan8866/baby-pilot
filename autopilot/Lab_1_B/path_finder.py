import heapq

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g_cost = float('inf')  # Cost from start to this node
        self.h_cost = 0  # Heuristic cost (estimated cost to goal)
        self.parent = None  # Parent node in the path

    def __lt__(self, other):
        # Compare nodes based on their f_cost (g_cost + h_cost)
        return (self.g_cost + self.h_cost) < (other.g_cost + other.h_cost)

def heuristic(node, goal):
    # Calculate the Manhattan distance as the heuristic cost
    return abs(node.x - goal.x) + abs(node.y - goal.y)

def a_star_search(grid, start, goal):
    open_list = []
    closed_set = set()

    start.g_cost = 0
    start.h_cost = heuristic(start, goal)
    heapq.heappush(open_list, start)

    while open_list:
        current_node = heapq.heappop(open_list)

        if current_node == goal:
            # Reconstruct and return the path
            path = []
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]

        closed_set.add(current_node)

        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            neighbor_x = current_node.x + dx
            neighbor_y = current_node.y + dy

            if 0 <= neighbor_x < len(grid) and 0 <= neighbor_y < len(grid[0]) and grid[neighbor_x][neighbor_y] == 0:
                neighbor_node = Node(neighbor_x, neighbor_y)
                if neighbor_node in closed_set:
                    continue

                tentative_g_cost = current_node.g_cost + 1  # Assuming each step has a cost of 1

                if tentative_g_cost < neighbor_node.g_cost:
                    neighbor_node.parent = current_node
                    neighbor_node.g_cost = tentative_g_cost
                    neighbor_node.h_cost = heuristic(neighbor_node, goal)

                    if neighbor_node not in open_list:
                        heapq.heappush(open_list, neighbor_node)

    return None  # No path found

