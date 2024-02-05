import heapq
from itertools import count
import settings

GRID_SIZE = settings.GRID_SIZE


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
    # for tie breaking, multiply h by 1+p, where p < 1/expected maximum path length
    return (1 + 1 / (4 * 50)) * (abs(start.x - goal.x) + abs(start.y - goal.y))


# use euclidean for > 8 directions
def euclidean_distance(start, goal):
    return ((start.x - goal.x)**2 + (start.y - goal.y)**2)**0.5


def get_neighbors(node, grid):
    '''
    Gets the neighbours of node in 8 directions
    '''
    directions = [(0, 1), (1, 1), (-1, 1), (1, -1), (-1, -1), (1, 0), (0, -1), (-1, 0)]  # Up, Right, Down, Left
    neighbors = []
    for dx, dy in directions:
        x, y = node.x + dx, node.y + dy
        if 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x, y] == 0:
            neighbors.append(Node(x, y))
    return neighbors


def a_star_search(grid, start, goal):
    open_set = []
    # set h of start
    start.f = euclidean_distance(start, goal)  # f = g + h = 0 + manhattan_distance
    counter = count()  # use as secondary sorting criterion for primary queue for tie breaking
    heapq.heappush(open_set, (start.f, next(counter), start))
    closed_set = set()
    if grid[goal.x][goal.y] == 1:
        print("Goal point is an obstacle. Unable to reach it.")
        return None
    while open_set:
        current_f, _, current_node = heapq.heappop(open_set)
        closed_set.add((current_node.x, current_node.y))
        if current_node == goal:
            path = []
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]

        for neighbor in get_neighbors(current_node, grid):
            if (neighbor.x, neighbor.y) in closed_set:
                continue
            neighbor.g = current_node.g + 1
            neighbor.h = euclidean_distance(neighbor, goal)
            neighbor.f = neighbor.g + neighbor.h
            neighbor.parent = current_node

            if add_to_open(open_set, neighbor, counter):
                heapq.heappush(open_set, (neighbor.f, next(counter), neighbor))
    print("No path found.")
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
