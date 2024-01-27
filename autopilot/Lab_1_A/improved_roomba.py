import picar_4wd as fc

# obstacle avoidance: go straight until sense object and circumvent it

import time

speed = 10
turn_duration = 2  # calculated by taking car_length / speed


def gear(is_drive: bool):
    if is_drive:
        fc.forward(speed)
    else:
        fc.stop()


def compute_turn_direction(scan_list: list) -> str:
    # figure out what direction to turn (90 deg)
    # if either side is not 2, turn right
    if scan_list[:3] == [2, 2, 2] and scan_list[7:] == [2, 2, 2]:
        fc.turn_right(speed)
        return "right"
    elif scan_list[7:] == [2, 2, 2]:
        print("turning right")
        fc.turn_right(speed)
        return "right"
    elif scan_list[:3] == [2, 2, 2]:
        print("turning left")
        fc.turn_left(speed)
        return "left"
    else: # blocked in
        print("boxed in")
        fc.backward(speed)
        new_scan_list = get_scan_list()
        return compute_turn_direction(new_scan_list)


def straighten_left(scan_list):
    while any(s == 2 for s in scan_list[:3]):
        fc.turn_left(speed)
        # Remember to recheck the scan_list after each turn
        scan_list = get_scan_list()


def straighten_right(scan_list):
    # turn right
    while any(s == 2 for s in scan_list[8:]):
        fc.turn_right(speed)
        # Remember to recheck the scan_list after each turn
        scan_list = get_scan_list()


def get_scan_list():
    scan_list = []
    while not scan_list:
        scan_list = fc.scan_step(50)
    return scan_list.copy()


def main():
    turn_start_time = 0
    # direction = "forward"
    while True:

        scan_list = fc.scan_step(50)
        print(turn_start_time, scan_list)
        if not scan_list or len(scan_list) != 10:  # wait for a full sweep before driving off
            continue

        tmp = scan_list[3:7]
        print("tmp", scan_list, tmp)

        if tmp != [2, 2, 2, 2]:
            # if direction == "forward":
            print("Encountered obstacle")
            fc.stop()
            direction = compute_turn_direction(scan_list.copy())
            # turn_start_time = time.time()
            # else keep turning
        else:
            fc.forward(speed) # move forward in current direction
            # if direction != "forward":
            #     # measure time since last loop here
            #     elapsed_time = time.time() - turn_start_time
            #     if elapsed_time >= turn_duration:
            #         print("turning back to original heading")
            #         if direction == "right":
            #             straighten_left(scan_list.copy())
            #             direction = "forward"
            #         elif direction == "left":
            #             straighten_right(scan_list.copy())
            #             direction = "forward"

if __name__ == "__main__":
    try:
        main()
    finally:
        fc.stop()
