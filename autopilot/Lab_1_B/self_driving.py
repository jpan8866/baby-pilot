from advanced_mapping import scan_environment
from vehicle_control import drive
from utils import mark_path_on_grid
import picar_4wd as fc
import path_finder
import numpy as np
import settings
import math
import time

CAR_POS = (settings.GRID_SIZE//2, 0)


def calculate_angle(current_point, prev_point) -> float:
    # Calculate the angle between the current point and the next point
    delta_x = current_point[0] - prev_point[0]
    delta_y = current_point[1] - prev_point[1]
    angle_radians = math.atan2(delta_x, delta_y)
    angle_degrees = math.degrees(angle_radians)
    # Convert the angle to the range (-180, 180]
    # if delta_x == 0:
    #     angle_degrees -= 90
    # elif delta_x < 0:
    #     angle_degrees = angle_degrees * -1 + 90 # Convert to negative angle
    # elif delta_x > 0:
    #     if delta_y < 0:
    #         angle_degrees = angle_degrees * -1 + 360
    #     else:
    #         angle_degrees = 90 - angle_degrees

    return angle_degrees


def follow_path(path, sleep_factor=0.05, power=10):
    # Follow the path using the car
    print("following path....")
    fc.start_speed_thread()
    i = 0
    prev_angle = 0
    angle = 0
    prev_point = (0, 0)
    try:
        while i < len(path) - 1:
            current_point = path[i]
            print("current point: ", current_point)
            # prev_point = path[i - 1]
            # print("previous point: ", prev_point)

            # Calculate the angle between the current point and the next point
            if i > 0:
                angle = calculate_angle(current_point, prev_point)
                print("previous point: ", prev_point)
                print("Current heading: ", prev_angle)
                print("Next heading: ", angle)

            # Count consecutive points in the same direction
            consecutive_points = 0
            print(calculate_angle(path[i + 1], current_point))
            while i < len(path) - 2 and angle == calculate_angle(path[i + 1], current_point):
                i += 1
                consecutive_points += 1
            prev_point = path[i]
            print("Distance to travel (cm): ", consecutive_points)

            # Turn the car to the calculated angle if needed
            if angle - prev_angle > 0:
                fc.turn_right(power)  # Adjust the power as needed
                time.sleep(abs(angle) * 0.01 * 0.8)
                print("Turned right by ", abs(angle))
            elif angle - prev_angle < 0:
                fc.turn_left(power)  # Adjust the power as needed
                time.sleep(abs(angle) * 0.01 * 0.8)
                print("Turned left by ", abs(angle))
            fc.stop()
            prev_angle = angle
            print("moving forward...")

            drive(consecutive_points)

            # # Move the car forward to the next point
            # fc.forward(power)  # Adjust the power as needed
            #
            # # Sleep for a factor of the number of consecutive points skipped
            # time.sleep(consecutive_points * sleep_factor) if consecutive_points > 0 else time.sleep(sleep_factor)

            # Stop the car
            fc.stop()
            print("stopped. Changing directions...")
            time.sleep(sleep_factor)

            i += 1

    finally:
        fc.stop()


def route():
    scanned_grid = scan_environment()
    np.set_printoptions(threshold=np.inf, linewidth=np.inf)

    # Define the start and goal
    start = path_finder.Node(settings.GRID_SIZE // 2, 0)
    goal = path_finder.Node(settings.GRID_SIZE // 2, settings.GRID_SIZE - 1)

    # Run A* algorithm
    path = path_finder.a_star_search_4dir(scanned_grid, start, goal)
    print("Path: ", path)
    print("Path length: ", len(path))

    # visualize map
    if path:
        mark_path_on_grid(scanned_grid, path)
    np.savetxt('./grid.txt', scanned_grid, fmt='%d')  # scp file to laptop to view

    # Visualize the grid
    # visualize_grid(scanned_grid, path, start, goal)

    # Follow the path
    follow_path(path)


if __name__ == '__main__':
    route()