import argparse
import picar_4wd as fc
import time
import vehicle_control

# def move_car(t: int):
#     try:
#         fc.forward(1)
#         time.sleep(t)
#         fc.stop()
#     finally:
#         print(1)
#         fc.stop()

def move_car(x: int):
    vehicle_control.drive_calculated(x)

if __name__ == "__main__":
    # parser = argparse.ArgumentParser(description="sleep time?")
    parser = argparse.ArgumentParser(description="Distance to drive")
    # parser.add_argument("t", type=int, help="amount of time to sleep")
    parser.add_argument("x", type=int, help="distance to drive")
    args = parser.parse_args()

    move_car(args.x)
