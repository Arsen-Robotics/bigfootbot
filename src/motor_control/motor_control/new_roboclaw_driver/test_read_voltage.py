# test_read_voltage.py

from roboclaw import Roboclaw

def main():
    # Initialize the Roboclaw driver
    roboclaw = Roboclaw("/dev/roboclaw", 38400)
    
    # Attempt to open the connection to the RoboClaw
    if not roboclaw.open():
        print("Failed to connect to RoboClaw.")
        return

    address = 0x80  # Set your RoboClaw's address here

    try:
        # Read the main battery voltage
        main_voltage = roboclaw.read_main_battery_voltage(address)
        if main_voltage is not None:
            print(f"Main battery voltage: {main_voltage / 10} V")
        else:
            print("Failed to read main battery voltage.")

        # Read the logic battery voltage
        logic_voltage = roboclaw.read_logic_battery_voltage(address)
        if logic_voltage is not None:
            print(f"Logic battery voltage: {logic_voltage / 10} V")
        else:
            print("Failed to read logic battery voltage.")

        # Read motor currents
        motor_currents = roboclaw.read_motor_currents(address)
        if motor_currents is not None:
            m1_current, m2_current = motor_currents
            print(f"Motor 1 current: {m1_current / 100.0} A")
            print(f"Motor 2 current: {m2_current / 100.0} A")
        else:
            print("Failed to read motor currents.")

        # Read the controller temperature
        temperature = roboclaw.read_temperature(address)
        if temperature is not None:
            print(f"Controller temperature: {temperature / 10} °C")
        else:
            print("Failed to read controller temperature.")

    except Exception as e:
        print(f"An error occurred: {e}")
    
    finally:
        # Close the connection to the RoboClaw
        roboclaw.close()

if __name__ == "__main__":
    main()
