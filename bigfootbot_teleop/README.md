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

4. Enter the following commands in the `bluetoothctl` prompt:

   ```
   power on
   agent on
   default-agent
   scan on
   ```


5. While the scan is active, put the PS3 controller into pairing mode by pressing and holding the "PS" button and the "Share" button on the controller simultaneously for a few seconds.

6. Wait for the PS3 controller to appear in the list of discovered devices. It should be listed as "Wireless Controller" or similar.

7. Stop the scan by entering `scan off` in the `bluetoothctl` prompt.

8. Pair and connect the PS3 controller by entering the following commands, replacing `<mac_address>` with the actual MAC address of the PS3 controller:

   ```
   pair <mac_address>
   connect <mac_address>
   ```


9. If the pairing and connection are successful, you should see a confirmation message.

10. Exit the `bluetoothctl` prompt by entering `exit`.
Once the PS3 controller is connected via Bluetooth, you can use it as an input device for games or other applications on your Linux system.
Please note that the exact steps and commands may vary slightly depending on your Linux distribution and Bluetooth adapter.



