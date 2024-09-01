# test_drive_commands.py

from roboclaw import Roboclaw  # Make sure roboclaw.py is in the same directory
import time

def main():
    # Initialize the Roboclaw driver
    roboclaw = Roboclaw("/dev/roboclaw", 38400)
    
    # Attempt to open the connection to the RoboClaw
    if not roboclaw.open():
        print("Failed to connect to RoboClaw.")
        return

    address = 0x80  # Set your RoboClaw's address here

    rate_hz = 40
    interval = 1.0 / rate_hz  # Calculate interval in seconds

    try:
        while True:
            start_time = time.time()

            # Send forward command to Motor 1 at half speed
            if not roboclaw.forward_m1(address, 10):
                print("Motor 1 Forward Command Failed")

            # Send forward command to Motor 2 at full speed
            if not roboclaw.forward_m2(address, 10):
                print("Motor 2 Forward Command Failed")

            # Calculate how much time has passed
            elapsed_time = time.time() - start_time
            time_to_wait = interval - elapsed_time

            # Sleep for the remaining time to maintain the 40 Hz rate
            if time_to_wait > 0:
                time.sleep(time_to_wait)

    except KeyboardInterrupt:
        print("Motor command loop interrupted by user.")

    # # Test driving motor 1 backward at full speed
    # print("Testing Motor 1 Backward (Full Speed)")
    # if roboclaw.backward_m1(address, 127):
    #     print("Motor 1 Backward Command Successful")
    # else:
    #     print("Motor 1 Backward Command Failed")

    # # Test driving motor 2 forward at full speed
    # print("Testing Motor 2 Forward (Full Speed)")
    # if roboclaw.forward_m2(address, 127):
    #     print("Motor 2 Forward Command Successful")
    # else:
    #     print("Motor 2 Forward Command Failed")

    # # Test driving motor 2 backward at half speed
    # print("Testing Motor 2 Backward (Half Speed)")
    # if roboclaw.backward_m2(address, 64):
    #     print("Motor 2 Backward Command Successful")
    # else:
    #     print("Motor 2 Backward Command Failed")

if __name__ == "__main__":
    main()
