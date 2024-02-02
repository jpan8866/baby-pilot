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
        print("%smm/s" % s)
    print("%smm" % x)
    speed.deinit()
    fc.stop()

if __name__ == "__main__":
    test()