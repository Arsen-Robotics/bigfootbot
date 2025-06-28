#!/bin/bash

# Confirmation prompt
read -p "Are you sure you want to run this script? (y/n): " confirm
if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
    echo "Exiting script. No changes were made."
    exit 1
fi

# Add CUDA versions to .bashrc
echo 'export PATH=/usr/local/cuda-11.4/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc

# Update and install dependencies
echo "Updating system and installing dependencies..."
sudo apt update && sudo apt upgrade -y
sudo apt install -y git-all jq v4l-utils v4l2loopback-dkms nano python3-pip

# Install JTOP
sudo -H pip3 install -U jetson-stats
sudo systemctl restart jtop.service

# Create partition and format to ext4
sudo wipefs --all /dev/nvme0n1
sudo sgdisk --zap-all /dev/nvme0n1
sudo parted -s /dev/nvme0n1 mklabel gpt
sudo parted -s /dev/nvme0n1 mkpart primary ext4 0% 100%
sleep 2
sudo wipefs --all /dev/nvme0n1p1
sudo mkfs.ext4 -F /dev/nvme0n1p1

# Mount SSD
echo "Mounting the SSD..."
sudo mkdir /mnt/nvme
sudo mount /dev/nvme0n1p1 /mnt/nvme
sudo chown -R bigfootbot:bigfootbot /mnt/nvme

echo "Adding entry to FSTAB..."
echo "/dev/nvme0n1p1  /mnt/nvme  ext4  defaults  0  2" | sudo tee -a /etc/fstab > /dev/null

# Delete existing folders on the SSD to avoid conflicts
echo "Deleting existing folders on the SSD..."
sudo rm -rf /mnt/nvme/*

# Create ROS2 workspace
echo "Creating ROS2 workspace..."
mkdir -p /mnt/nvme/ros2_ws/src

# Navigate to the workspace and clone the repository
cd /mnt/nvme/ros2_ws/src
echo "Cloning bigfootbot repository..."
git clone https://github.com/Arsen-Robotics/bigfootbot.git

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

# # Install Docker
# echo "Installing Docker..."

# # Uninstall conflicting packages
# for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do sudo apt-get remove $pkg; done

# # Add Docker's official GPG key:
# sudo apt-get update
# sudo apt-get install -y ca-certificates curl
# sudo install -m 0755 -d /etc/apt/keyrings
# sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
# sudo chmod a+r /etc/apt/keyrings/docker.asc

# # Add the repository to Apt sources:
# echo \
# "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
# $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
# sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
# sudo apt-get update

# # Install Docker
# sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# echo "Docker installed successfully!"

# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

# Install docker compose plugin
sudo apt install docker-compose-plugin

# Add the current user to the Docker group
echo "Adding user to the Docker group..."
sudo usermod -aG docker ${USER}

# Move the Docker data folder to SSD
echo "Configuring Docker data location..."
sudo mkdir /mnt/nvme/docker
sudo chown root:docker /mnt/nvme/docker
sudo systemctl stop docker
sudo rsync -aP /var/lib/docker/ /mnt/nvme/docker
sudo rm -rf /var/lib/docker
sudo jq '. + {"data-root": "/mnt/nvme/docker"}' /etc/docker/daemon.json > temp.json && sudo mv temp.json /etc/docker/daemon.json
sudo systemctl start docker

# Configure NVIDIA Docker runtime
echo "Configuring NVIDIA Docker runtime"
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Hold the pre-installed NVIDIA Docker packages to prevent overwrite by apt upgrade
sudo apt-mark hold docker.io containerd docker-buildx-plugin docker-compose-plugin nvidia-container-toolkit nvidia-docker2

# Udev rules
echo "Installing necessary Udev rules..."

# Copy udev rules
sudo cp /mnt/nvme/ros2_ws/src/bigfootbot/src/motor_control/udev/99-roboclaw.rules /etc/udev/rules.d
sudo cp /mnt/nvme/ros2_ws/src/bigfootbot/src/bfb_gps/udev/99-gps-module.rules /etc/udev/rules.d
sudo cp /mnt/nvme/ros2_ws/src/bigfootbot/src/bfb_arduino_gateway/udev/99-arduino-mega.rules /etc/udev/rules.d
sudo cp /mnt/nvme/ros2_ws/src/bigfootbot/docker/web/webrtc_video/udev/99-cameras.rules /etc/udev/rules.d
# sudo cp /mnt/nvme/ros2_ws/src/bigfootbot/docker/web/transitive_robotics/udev/99-usb-cameras.rules /etc/udev/rules.d
# sudo cp ~/ros2_ws/src/bigfootbot/docker/web/transitive_robotics/udev/99-usb-cameras.rules /etc/udev/rules.d

# Reload rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Add display environment variable for Nvidia acceleration
echo 'export DISPLAY=:0' >> ~/.bashrc
echo 'xhost +local:root' >> ~/.bashrc

# Change NVIDIA fan control profile to "cool"
sudo systemctl stop nvfancontrol
sudo sed -i.bak -E "s/^(.*FAN_DEFAULT_PROFILE[[:space:]]+)(quiet|cool)(.*)$/\1cool\3/" /etc/nvfancontrol.conf
sudo rm /var/lib/nvfancontrol/status
sudo systemctl start nvfancontrol

# Notify user to setup Docker network
echo "After reboot, follow instructions in the end of this file."

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

# ---------- 1. Set up Docker network -------------

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

# ---------- 2. Enable Jetson Clocks on boot in jtop -------------
