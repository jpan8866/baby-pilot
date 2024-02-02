import argparse
from vehicle_control import drive, turn

FEET_TO_DISTANCE_UNIT = 15
TURNING_POWER = 1
DRIVING_POWER = 10

def move_car(x: int, y: int):
    # Convert feet to distance parameter
    x_distance = abs(x) * FEET_TO_DISTANCE_UNIT
    y_distance = abs(y) * FEET_TO_DISTANCE_UNIT

    # Move horizontally
    if x_distance > 0:
        turn(angle=90, power=TURNING_POWER)
    else:     
        turn(angle=-90, power=TURNING_POWER)
    drive(distance=x_distance, power=DRIVING_POWER)

    # Move vertically
    if y_distance < 0:
        turn(angle=180, power=TURNING_POWER)
    drive(distance=y_distance, power=DRIVING_POWER)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Drive a car to a specific (x, y) location.")
    parser.add_argument("x", type=int, help="X coordinate (feet), positive for right, negative for left")
    parser.add_argument("y", type=int, help="Y coordinate (feet), positive for forwards, negative for backwards")
    
    args = parser.parse_args()

    move_car(args.x, args.y)