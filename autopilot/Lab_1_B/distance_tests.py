from picar_4wd.speed import *
import picar_4wd as fc

def test():
    speed = Speed(25)
    speed.start()
    print(speed)
    fc.forward(10)
    x = 0
    for i in range(20):
        time.sleep(0.1)
        s = speed()
        x += s * 0.1
        print("%scm/s" % s)
    print("%scm" % x)
    speed.deinit()
    fc.stop()


def drive(distance: int, power: int=10):
    fc.start_speed_thread()
    x = 0
    fc.forward(power)
    while x < distance*0.95:
        time.sleep(0.1)
        s = fc.speed_val()
        x += s * 0.1
        print("%scm" % x)
    fc.left_rear_speed.deinit()
    fc.right_rear_speed.denit()
    fc.stop()


if __name__ == "__main__":
    drive(15.75)  # length of an iphone xs max for testing
