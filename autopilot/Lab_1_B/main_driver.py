import argparse
import math
from vehicle_control import drive, turn

FEET_TO_DISTANCE_UNIT = 30
LOSS_COMPENSATION = 2
TURNING_POWER = 5
DRIVING_POWER = 10

def calculate_turn_and_distance(x, y):
    angle_radians = math.atan2(y, x)
    angle_degrees = math.degrees(angle_radians) * LOSS_COMPENSATION
    distance = math.sqrt(x**2 + y**2) * FEET_TO_DISTANCE_UNIT
    
    return angle_degrees, distance

def move_car(x: int, y: int):
    angle_degrees, distance = calculate_turn_and_distance(x, y)
    turn(angle=angle_degrees, power=TURNING_POWER)
    drive(distance, power=DRIVING_POWER)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Drive a car to a specific (x, y) location.")
    parser.add_argument("x", type=int, help="X coordinate (feet), positive for right, negative for left")
    parser.add_argument("y", type=int, help="Y coordinate (feet), positive for forwards, negative for backwards")
    
    args = parser.parse_args()

    move_car(args.x, args.y)
