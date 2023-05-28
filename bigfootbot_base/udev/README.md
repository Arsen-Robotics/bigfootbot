# Udev Rules for Roboclaw Motor Controller

This folder contains the Udev rules file (`99-roboclaw.rules`) for the Roboclaw motor controller. The Udev rules allow the system to assign a consistent device name to the Roboclaw when it is connected to the computer.

## Purpose

The Udev rules file ensures that the Roboclaw motor controller is assigned a predictable device name, making it easier to identify and interact with the device from your application or software. By providing a consistent device name (e.g., `/dev/roboclaw`), you can access the motor controller reliably without relying on changing device identifiers.

## Usage

To use the Udev rules for the Roboclaw motor controller, follow these steps:

1. Copy the `99-roboclaw.rules` file from this folder to the Udev rules directory. You   
   can do this by running the following command:

   ```bash
   sudo cp 99-roboclaw.rules /etc/udev/rules.d/
   ```

2. Reload the Udev rules by running the following command:

   ```bash
   sudo udevadm control --reload-rules
   ```
   
   This ensures that any changes made to the Udev rules are taken into account.

3. Trigger Udev to reapply the rules by running the following command:

   ```bash
   sudo udevadm trigger
   ``

   This will immediately apply the Udev rules, and the Roboclaw motor controller should now be accessible with the assigned device name.

## Notes

- Make sure to run the above commands with administrative privileges using `sudo`.
- After applying the Udev rules, reconnect the Roboclaw motor controller to the computer to ensure the new device name is assigned.

