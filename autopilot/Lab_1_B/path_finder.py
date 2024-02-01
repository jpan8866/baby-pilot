import heapq
from itertools import count

class Node:
    def __init__(self, x, y, g=0, h=0, f=0, parent=None):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.f = f
        self.parent = parent

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

# Heuristics
# use manhattan for 4 directions
def manhattan_distance(start, goal):
    return abs(start.x - goal.x) + abs(start.y - goal.y)

# use euclidean for > 8 directions
def euclidean_distance(start, goal):
    return ((start.x - goal.x)**2 + (start.y - goal.y)**2)**0.5

def get_neighbors_4dir(node, grid):
    '''
    Gets the neighbours of node in 4 directions
    '''
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Up, Right, Down, Left
    neighbors = []
    for dx, dy in directions:
        x, y = node.x + dx, node.y + dy
        if 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x, y] == 0:
            neighbors.append(Node(x, y))
    return neighbors


def a_star_search_4dir(grid, start, goal):
    open_set = []
    # set h of start
    start.f = manhattan_distance(start, goal)  # f = g + h = 0 + manhattan_distance
    counter = count()  # use as secondary sorting criterion for primary queue
    heapq.heappush(open_set, (start.f, next(counter), start))
    closed_set = set()

    while open_set:
        current_f, _, current_node = heapq.heappop(open_set)
        closed_set.add((current_node.x, current_node.y))
        print(open_set, current_f, current_node)
        if current_node == goal:
            path = []
            print("found goal")
            print(path, current_node)
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]

        for neighbor in get_neighbors_4dir(current_node, grid):
            if (neighbor.x, neighbor.y) in closed_set:
                continue
            neighbor.g = current_node.g + 1
            neighbor.h = manhattan_distance(neighbor, goal)
            neighbor.f = neighbor.g + neighbor.h

            if add_to_open(open_set, neighbor, counter):
                heapq.heappush(open_set, (neighbor.f, next(counter), neighbor))

    return None

def add_to_open(open_set, neighbor, counter):
    # open_set contains next nodes to visit
    for i, (f, _, node) in enumerate(open_set):
        if neighbor == node:
            # if neighbor is already part of next nodes to visit (e.g. added thru another path
            # and that it cost less than what it currently does, then replace with new path cuz lesser cost
            # if it does not cost less, do not consider this path -> ignore neighbor
            if neighbor.g < node.g:
                open_set[i] = (neighbor.f, next(counter), neighbor)
            return False
    return True

#
# def heuristic(node, goal):
#     # Calculate the Manhattan distance as the heuristic cost
#     return abs(node.x - goal.x) + abs(node.y - goal.y)
#
# def a_star_search(grid, start, goal):
#     open_list = []
#     closed_set = set()
#
#     start.g_cost = 0
#     start.h_cost = heuristic(start, goal)
#     heapq.heappush(open_list, start)
#     print("started A* search")
#
#     while open_list:
#         print("current path: ")
#         print(path)
#         current_node = heapq.heappop(open_list)
#
#         if current_node == goal:
#             # Reconstruct and return the path
#             path = []
#             while current_node:
#                 path.append((current_node.x, current_node.y))
#                 current_node = current_node.parent
#             return path[::-1]
#
#         closed_set.add(current_node)
#
#         # for each neighbour of current_node
#         for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
#             neighbor_x = current_node.x + dx
#             neighbor_y = current_node.y + dy
#
#             if 0 <= neighbor_x < len(grid) and 0 <= neighbor_y < len(grid[0]) and grid[neighbor_x][neighbor_y] == 0:
#                 neighbor_node = Node(neighbor_x, neighbor_y)
#                 if neighbor_node in closed_set:
#                     continue
#
#                 tentative_g_cost = current_node.g_cost + 1  # Assuming each step has a cost of 1
#
#                 if tentative_g_cost < neighbor_node.g_cost:
#                     neighbor_node.parent = current_node
#                     neighbor_node.g_cost = tentative_g_cost
#                     neighbor_node.h_cost = heuristic(neighbor_node, goal)
#
#                     if neighbor_node not in open_list:
#                         heapq.heappush(open_list, neighbor_node)
#
#     return None  # No path found
#
