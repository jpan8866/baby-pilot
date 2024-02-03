from advanced_mapping import scan_environment
from path_finder import a_star_search_4dir as astar
import picar_4wd as fc
import numpy as np
import settings
import math
import time

CAR_POS = (settings.GRID_SIZE//2, 0)


def calculate_angle(current_point, next_point) -> float:
    # Calculate the angle between the current point and the next point
    delta_x = next_point[0] - current_point[0]
    delta_y = next_point[1] - current_point[1]
    angle_radians = math.atan2(delta_y, delta_x)
    angle_degrees = math.degrees(angle_radians)
    # Convert the angle to the range (-180, 180]
    if delta_x == 0:
        angle_degrees -= 90
    elif delta_x < 0:
        angle_degrees = angle_degrees * -1 + 90 # Convert to negative angle
    elif delta_x > 0:
        if delta_y < 0:
            angle_degrees = angle_degrees * -1 + 360
        else:
            angle_degrees = 90 - angle_degrees

    return angle_degrees


def follow_path(path, sleep_factor=0.05, power=10):
    # Follow the path using the car
    print("following path....")
    i = 0
    try:
        while i < len(path) - 1:
            current_point = path[i]
            print("current point: ", current_point)
            next_point = path[i + 1]
            print("next point: ", next_point)

            # Calculate the angle between the current point and the next point
            angle = calculate_angle(current_point, next_point)
            print("turn angle: ", angle)

            # Count consecutive points in the same direction
            consecutive_points = 0
            while angle == calculate_angle(current_point, path[i + 1]) and i < len(path) - 2:
                i += 1
                next_point = path[i + 1]
                consecutive_points += 1
            print("consecutive points same direction: ", consecutive_points)

            # Turn the car to the calculated angle if needed
            if angle > 0:
                fc.turn_right(power)  # Adjust the power as needed
                time.sleep(abs(angle) * 0.01 * 0.8)
            elif angle < 0:
                fc.turn_left(power)  # Adjust the power as needed
                time.sleep(abs(angle) * 0.01 * 0.8)
            print("turned.... if needed")
            print("moving forward...")

            # Move the car forward to the next point
            fc.forward(power)  # Adjust the power as needed

            # Sleep for a factor of the number of consecutive points skipped
            time.sleep(consecutive_points * sleep_factor) if consecutive_points > 0 else time.sleep(sleep_factor)

            # Stop the car
            fc.stop()
            print("stopped. will change directions...")
            time.sleep(sleep_factor)

            i += 1

    finally:
        fc.stop()


def route():
    scanned_grid = scan_environment()
    np.set_printoptions(threshold=np.inf, linewidth=np.inf)

    # Define the start and goal

    start = CAR_POS
    goal = (settings.GRID_SIZE//2, settings.GRID_SIZE)

    # Run A* algorithm
    path = astar(scanned_grid, start, goal)  # TODO factor in a boundary buffer for obstacles
    print("Path: ", path)
    print("Path length: ", len(path))

    # Visualize the grid
    # visualize_grid(scanned_grid, path, start, goal)

    # Follow the path
    follow_path(path)
