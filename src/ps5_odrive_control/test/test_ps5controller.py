from inputs import get_gamepad

def main():
    print("Press buttons or move sticks on your PS5 controller...")

    try:
        # Repeat
        while True:
            events = get_gamepad()
            for event in events:
                # event.code = button or axis name
                # event.state = value (0/1 for buttons, -32768..32767 for sticks)
                if event.code != "SYN_REPORT":
                    print(f"{event.code}: {event.state}")

        # Do once
        # events = get_gamepad()
        # for event in events:
        #     # event.code = button or axis name
        #     # event.state = value (0/1 for buttons, -32768..32767 for sticks)
        #     print(f"{event.code}: {event.state}")
            
    except KeyboardInterrupt:
        print("Exiting...")

if __name__ == "__main__":
    main()
