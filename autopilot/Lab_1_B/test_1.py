import argparse
import picar_4wd as fc
import time

def move_car(t: int):
    try:
        fc.forward(1)
        time.sleep(t)
        fc.stop()
    finally:
        print(1)
        fc.stop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="sleep time?")
    parser.add_argument("t", type=int, help="amount of time to sleep")

    args = parser.parse_args()

    move_car(args.t)