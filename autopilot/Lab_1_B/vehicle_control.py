from picar_4wd.speed import *
import picar_4wd as fc
import argparse


def drive(distance: int, power: int = 10):
    '''
    Calculates the speed using the Photo Interruptor
    Given a distance, we drive until distance is met.
    '''
    fc.start_speed_thread()
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


# def turn_diagonal(angle: int):
#     '''
#     stop car first before initiating turn
#     experiment with gradually scaling up acceleration
#     experiment with slower speed
#     '''
#     fc.stop()
#     time.sleep(0.1)
#     fc.turn_left(2)
#     time.sleep(0.5)
#     fc.turn_left(4)
#     time.sleep(0.5)
#     fc.turn_left(6)
#     fc.stop()


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

