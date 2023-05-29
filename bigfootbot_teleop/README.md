# Connecting PS3 Controller to PC via Bluetooth (Linux CLI)

This guide provides instructions on how to connect a PlayStation 3 (PS3) controller to your PC using the Linux command-line interface (CLI).

## Prerequisites

- Make sure your computer has Bluetooth capabilities and that Bluetooth is enabled.

## Steps

1. Open a terminal on your Linux system.

2. Install the required dependencies by running the following command:

    ```
    sudo apt-get install bluez
    ```


3. Run the `bluetoothctl` command to start the Bluetooth control tool:

    ```
    bluetoothctl
    ```

4. Press and hold the "PS" button on your PS3 DualShock 3 controller until the LEDs start blinking rapidly. 
   This puts the controller into pairing mode.

5. Enter the following commands in the `bluetoothctl` prompt:

   ```
   power on
   agent on
   default-agent
   scan on
   ```

6. Wait for the PS3 DualShock 3 controller to appear in the list of discovered devices. It should be listed as "Wireless Controller" or similar.

7. Once the controller is discovered, note its MAC address.

8. Stop the scan by entering `scan off` in the `bluetoothctl` prompt.

9. Pair and connect the PS3 controller by entering the following commands, replacing `<mac_address>` with the actual MAC address of the PS3 controller:

   ```
   pair <mac_address>
   ```
   ```
   connect <mac_address>
   ```

10. If the pairing and connection are successful, you should see a confirmation message.

11. Exit the `bluetoothctl` prompt by entering `exit`.
Once the PS3 controller is connected via Bluetooth, you can use it as an input device for games or other applications on your Linux system.
Please note that the exact steps and commands may vary slightly depending on your Linux distribution and Bluetooth adapter.



