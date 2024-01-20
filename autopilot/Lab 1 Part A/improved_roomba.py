import picar_4wd as fc

# obstacle avoidance: go straight until sense object and circumvent it

import time

speed = 30

def gear(is_drive: bool):
    if is_drive:
        fc.forward(speed)
    else:
        fc.stop()

def compute_turn_direction(scan_list: list):
    # figure out what direction to turn (90 deg)
    # if either side is not 2, turn right
    if scan_list[:3] == [2,2,2] and scan_list[8:] == [2,2,2]:
        fc.turn_right(speed)
    elif scan_list[:3] != [2,2,2]:
        print("turning right")
        # fc.turn_right(speed)
    else:
        print("turning left")
        # fc.turn_left(speed)


def main():
    while True:
        scan_list = fc.scan_step(35)

        if not scan_list:
            continue

        tmp = scan_list[3:7]
        print(tmp)

        if tmp != [2, 2, 2, 2]: # directly in front of car
            print("obstacle")
            gear(is_drive=False)  # Stop if obstacle detected
            compute_turn_direction(scan_list)
            # logic to check whether to turn right or left
            # straight until does not sense anything on either side + 2 more second to let whole body get accross
            # turn back same amount of seconds
            # turn back straight (optional)
        else:
            gear(is_drive=True)  # Start driving

if __name__ == "__main__":
    try:
        main()
    finally:
        fc.stop()
