# Udev Rules for Roboclaw Motor Controller

This folder contains the Udev rules file (`99-ps3-controller.rules`) for the Playstation 3 Controller. The Udev rules allow the system to assign a consistent device name to the PS3 Controller when it is connected to the computer.

## Purpose

The Udev rules file ensures that the Playstation 3 Controller is assigned a predictable device name, making it easier to identify and interact with the device from your application or software. By providing a consistent device name (e.g., `/dev/ps3_controller`), you can access the PS3 controller reliably without relying on changing device identifiers.

## Usage (manual installation)

To use the Udev rules for the PS2 Controller, follow these steps:

1. Copy the `99-ps3-controller.rules` file from this folder to the Udev rules directory. You   
   can do this by running the following command:

   ```bash
   sudo cp 99-ps3-controller.rules /etc/udev/rules.d/
   ```

2. Reload the Udev rules by running the following command:

   ```bash
   sudo udevadm control --reload-rules
   ```
   
   This ensures that any changes made to the Udev rules are taken into account.

3. Trigger Udev to reapply the rules by running the following command:

   ```bash
   sudo udevadm trigger
   ```

   This will immediately apply the Udev rules, and the PS3 Controller should now be accessible with the assigned device name.

## Usage (installation using script)

Alternatively, you can use the provided script in the script directory to apply the Udev rules automatically. The script simplifies the process of copying the rules file and reloading the rules. Follow these steps:

1. Navigate to the script directory:

   ```bash 
   cd script

2. Run the script to apply the Udev rules:
  
   ```bash
   sudo ./apply_udev_rules.sh

## Notes

- Make sure to run the above commands with administrative privileges using `sudo`.

