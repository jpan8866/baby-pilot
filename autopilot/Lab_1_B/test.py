import argparse
from vehicle_control import turn, turn_right
from advanced_mapping_brent import route
import picar_4wd as fc

FEET_TO_DISTANCE_UNIT = 30
LOSS_COMPENSATION = 2
TURNING_POWER = 5
DRIVING_POWER = 10
SHARP_TURN = 90
TURN_AROUND = 180


# x and y in meters
def move_car(x: int, y: int):
    # move forward first
    if(y < 0):
        turn(TURN_AROUND, TURNING_POWER) 
    for _ in range(y):
        route()
    
    if(x > 0):
        turn_right(SHARP_TURN, TURNING_POWER) 
    if(x < 0):
        turn(SHARP_TURN, TURNING_POWER)         
    for _ in range(x):
        route()
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Drive a car to a specific (x, y) location.")
    parser.add_argument("x", type=int, help="X coordinate (feet), positive for right, negative for left")
    parser.add_argument("y", type=int, help="Y coordinate (feet), positive for forwards, negative for backwards")
    
    args = parser.parse_args()

    move_car(args.x, args.y)
