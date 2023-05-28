#!/bin/bash

# Specify the path to the Udev rules file
RULES_FILE="../udev/99-roboclaw.rules"

# Check if the rules file exists
if [[ ! -f "$RULES_FILE" ]]; then
    echo "Udev rules file does not exist: $RULES_FILE"
    exit 1
fi

# Copy the rules file to the Udev rules directory
cp "$RULES_FILE" "/etc/udev/rules.d/"

# Check if the copy was successful
if [[ $? -ne 0 ]]; then
    echo "Failed to copy the Udev rules file"
    exit 1
fi

# Reload the Udev rules
sudo udevadm control --reload-rules

# Trigger Udev to reapply the rules
sudo udevadm trigger

# Check if the rules were successfully applied
if [[ $? -eq 0 ]]; then
    echo "Udev rules applied successfully"
else
    echo "Failed to apply Udev rules"
    exit 1
fi

exit 0