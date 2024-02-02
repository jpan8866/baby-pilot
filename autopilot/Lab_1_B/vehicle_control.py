from picar_4wd.speed import *
import picar_4wd as fc


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
        print("%scm" % x)
    fc.stop()
    fc.left_rear_speed.deinit()
    fc.right_rear_speed.deinit()


def turn(angle: int, power: int = 10):
    '''
    stop car first before initiating turn
    experiment with gradually scaling up acceleration
    experiment with slower speed
    '''
    fc.stop()
    time.sleep(0.1)
    turn_left(2)
    time.sleep(0.5)
    turn_left(4)
    time.sleep(0.5)
    turn_left(6)
    fc.stop()



if __name__ == "__main__":
    # drive(15.75)  # length of an iphone xs max for testing
    turn(90)
