#!/bin/bash

# Confirmation prompt
read -p "Are you sure you want to run this script? (y/n): " confirm
if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
    echo "Exiting script. No changes were made."
    exit 1
fi

# Update and install git
echo "Updating system and installing Git..."
sudo apt update && sudo apt upgrade -y
sudo apt install git-all -y

# Create ROS2 workspace
echo "Creating ROS2 workspace..."
mkdir -p ~/ros2_ws/src

# Navigate to the workspace and clone the repository
cd ~/ros2_ws/src
echo "Cloning bigfootbot repository..."
git clone https://github.com/arsenikalbin/bigfootbot.git

# Change to the develop branch
cd bigfootbot
echo "Switching to the 'develop' branch..."
git checkout develop

# Set Git username and email
echo "Configuring Git user..."

# Ask for the Git username
read -p "Enter your Git username: " git_username
git config --global user.name "$git_username"

# Ask for the Git email
read -p "Enter your Git email: " git_email
git config --global user.email "$git_email"

# Set up Git credential storage
echo "Storing Git credentials..."
git config --global credential.helper store
git push

# Install Docker
while true; do
    echo "Please choose your operating system:"
    echo "1. Ubuntu"
    echo "2. Debian"

    read -p "Enter your choice (number): " os_choice

    case $os_choice in
        1)
            echo "Installing Docker on Ubuntu..."

            # Uninstall conflicting packages
            for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do sudo apt-get remove $pkg; done

            # Add Docker's official GPG key:
            sudo apt-get update
            sudo apt-get install ca-certificates curl
            sudo install -m 0755 -d /etc/apt/keyrings
            sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
            sudo chmod a+r /etc/apt/keyrings/docker.asc

            # Add the repository to Apt sources:
            echo \
            "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
            $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
            sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
            sudo apt-get update

            # Install Docker
            sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

            echo "Docker installed successfully!"
            break
            ;;

        2)
            echo "Installing Docker on Debian..."

            # Uninstall conflicting packages
            for pkg in docker.io docker-doc docker-compose podman-docker containerd runc; do sudo apt-get remove $pkg; done

            # Add Docker's official GPG key:
            sudo apt-get update
            sudo apt-get install ca-certificates curl
            sudo install -m 0755 -d /etc/apt/keyrings
            sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
            sudo chmod a+r /etc/apt/keyrings/docker.asc

            # Add the repository to Apt sources:
            echo \
            "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
            $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
            sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
            sudo apt-get update

            # Install Docker
            sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

            echo "Docker installed successfully!"
            break
            ;;

        *)
            echo "Invalid choice. Please choose valid option."
            # The loop continues, prompting the user again
            ;;
    esac
done

# Add the current user to the Docker group
echo "Adding user to the Docker group..."
sudo usermod -aG docker ${USER}

# Udev rules
echo "Installing necessary Udev rules..."

# Copy udev rules
sudo cp ~/ros2_ws/src/bigfootbot/motor_control/udev/99-roboclaw.rules /etc/udev/rules.d
sudo cp ~/ros2_ws/src/bigfootbot/bfb_gps/udev/99-gps-module.rules /etc/udev/rules.d
sudo cp ~/ros2_ws/src/bigfootbot/bfb_arduino_gateway/udev/99-arduino-mega.rules /etc/udev/rules.d
# sudo cp ~/ros2_ws/src/bigfootbot/docker/web/transitive_robotics/udev/99-usb-cameras.rules /etc/udev/rules.d

# Reload rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Notify user to setup Docker network
echo "After reboot, follow instructions in the end of this file to setup Docker macvlan network."

while true; do
    echo "Do you want to reboot now?"
    echo "1. Yes"
    echo "2. No, I will reboot later"

    read -p "Enter your choice (number): " os_choice

    case $os_choice in
        1)
            echo "Rebooting now..."
            sudo reboot
            break
            ;;

        2)
            echo "Please reboot your device later to apply changes."
            break
            ;;

        *)
            echo "Invalid choice. Please choose valid option."
            # The loop continues, prompting the user again
            ;;

    esac
done

# --- Set up Docker network ---

# NB! Replace eth0 with your interface. You can list interfaces with command "ip a"

# docker network create -d macvlan \
#     --subnet=192.168.5.0/24 \
#     --gateway=192.168.5.1 \
#     --ip-range=192.168.5.64/27 \
#     --attachable \
#     -o parent=eth0 \
#     macnet

# sudo ip link add macnet-shim link eth0 type macvlan  mode bridge
# sudo ip addr add 192.168.5.64/27 dev macnet-shim
# sudo ip link set macnet-shim up

# docker run --rm -d --net=macnet <IMAGE>