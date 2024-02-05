from picar_4wd.speed import *
import picar_4wd as fc
import argparse
import math
import time


def drive(distance: int, power: int = 10) -> int:
    '''
    Calculates the speed using the Photo Interruptor
    Given a distance, we drive until distance is met.
    returns traveled distance
    '''
    x = 0
    fc.forward(power)
    while x < distance*0.95:
        time.sleep(0.05)
        s = fc.speed_val()
        x += s * 0.05
        # print("%scm" % x)
    fc.stop()
    fc.left_rear_speed.deinit()
    fc.right_rear_speed.deinit()
    return x


def turn(angle: int, power: int = 1):
    fc.stop()
    s = Speed(25)
    s.start()
    a = 0
    fc.turn_left(power)
    # scale up angle 40% to account for hops and grip loss
    while a < angle*1.4:
        time.sleep(0.05)
        speed = s()
        # Degrees turned = w * t = v/r * t = 2v/L * t. Multiply by 180/pi to get degrees
        a += 180/math.pi * 2 * speed * 0.05 / 17
    fc.stop()
    s.deinit()

def turn_right(angle: int, power: int = 1):
    fc.stop()
    s = Speed(25)
    s.start()
    a = 0
    fc.turn_right(power)
    # scale up angle 40% to account for hops and grip loss
    while a < angle*1.4:
        time.sleep(0.05)
        speed = s()
        # Degrees turned = w * t = v/r * t = 2v/L * t. Multiply by 180/pi to get degrees
        a += 180/math.pi * 2 * speed * 0.05 / 17
    fc.stop()
    s.deinit()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Turn given a direction, angle and power")
    parser.add_argument("dir", type=str, help="Direction (L/R)")
    parser.add_argument("angle", type=int, help="Angle")
    parser.add_argument("pow", type=int, help="Power")
    
    args = parser.parse_args()

    if args.dir == "L":
        turn(args.angle, args.pow)
    else:
        turn_right(args.angle, args.pow)


def drive_calculated(distance: int) -> int:
    '''
    calculated 26cm/s for power = 1
    '''
    speed = 25
    time_to_drive = distance/speed
    fc.forward(1)
    time.sleep(time_to_drive)
    fc.stop()