import picar_4wd as fc

# obstacle avoidance: go straight until sense object and determine direction to take

speed = 10
scan_ref = 45


def compute_turn_direction(scan_list: list) -> str:
    if scan_list[:3] == [2, 2, 2] and scan_list[7:] == [2, 2, 2]:
        print("both directions clear, turning right by default")
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
    else:
        # blocked in
        print("Boxed in, backing up and reassessing")
        fc.backward(speed)
        new_scan_list = get_scan_list()
        return compute_turn_direction(new_scan_list)


def get_scan_list():
    scan_list = []
    # get full length of scan_list before proceeding
    while not scan_list or len(scan_list) != 10:
        # use higher ref for more buffer distance to allow for full sweep of ultrasonic sensor
        scan_list = fc.scan_step(scan_ref)
    return scan_list.copy()


def main():
    while True:
        scan_list = get_scan_list()

        tmp = scan_list[3:7]
        print("tmp", scan_list, tmp)

        if tmp != [2, 2, 2, 2]:
            print("Encountered obstacle, tmp: ", tmp)
            fc.stop()
            compute_turn_direction(scan_list.copy())
        else:
            fc.forward(speed)  # move forward in current direction


if __name__ == "__main__":
    try:
        main()
    finally:
        fc.stop()
