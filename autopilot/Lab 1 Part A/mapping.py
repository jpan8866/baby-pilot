import random
import picar_4wd as fc
import time

speed = 30

def main():
    while True:
        scan_list = fc.scan_step(50)
        if not scan_list:
            continue

        tmp = scan_list[3:7]
        print(tmp)
        if tmp != [2,2,2,2]:
            # stop
            fc.stop()
            time.sleep(0.5)
            
            # back up a bit
            fc.backward(speed/4)
            time.sleep(0.5)
            
            # pick random direction and turn
            if random.randint(0,1) == 0:
                fc.turn_left(speed/4)
            else:
                fc.turn_right(speed/4)
        else:
            fc.forward(speed)

if __name__ == "__main__":
    try:
        main()
    finally:
        fc.stop()