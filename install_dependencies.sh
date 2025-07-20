#!/bin/bash

#   author: Nicolas Erbetti
#   brief: This file takes care of downloading and installing every package 
#          required by the web server laboratory.
#	   If the installation looks like it's stuck at some point, try to do CTL+D.
#    To start everything in a sourced workspace, use: ros2 launch launcher full_system.launch

################################
# PACKAGES INSTALLATION ########
################################

# Setup repositories.
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb

# Update the packages.
sudo apt update
sudo apt upgrade

# Install the ROS2 packages.
sudo apt install -y \
  ros-humble-desktop-full \
  ros-humble-turtlebot3* \
  ros-humble-rosbridge-server \
  ros-humble-apriltag-ros ros-humble-apriltag \
  python3-colcon-common-extensions \
  python3-pip

# Install the AI python packages. (May need to adapt the version of Cuda based on the GPU drivers).
python3 -m pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
python3 -m pip install ultralytics

################################
# CLOUDFLARE SETUP ########
################################

# Download sources.
sudo mkdir -p --mode=0755 /usr/share/keyrings
curl -fsSL https://pkg.cloudflare.com/cloudflare-main.gpg | sudo tee /usr/share/keyrings/cloudflare-main.gpg >/dev/null
echo "deb [signed-by=/usr/share/keyrings/cloudflare-main.gpg] https://pkg.cloudflare.com/cloudflared any main" | sudo tee /etc/apt/sources.list.d/cloudflared.list
sudo apt-get update && sudo apt-get install cloudflared

# Enable tunnel.
cloudflared tunnel login

################################
# BASHRC SETUP ########
################################

# Defint the ros distro to be sourced in every terminal.
echo 'source /opt/ros/humble/local_setup.bash' >> ~/.bashrc
echo 'source /home/nicolas/personal-website/server/ros2_lab_ws/install/local_setup.bash' >> ~/.bashrc
# Define the turtlebot model used in the simulation.
echo 'export TURTLEBOT3_MODEL=burger_cam' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/nicolas/personal-website/server/ros2_lab_ws/src/simulation/gazebo_apriltag/models' >> ~/.bashrc
echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/humble/lib' >> ~/.bashrc
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'source /usr/share/gazebo-11/setup.sh' >> ~/.bashrc
echo 'source /usr/share/gazebo-11/setup.bash' >> ~/.bashrc
# Source the bashrc.
source ~/.bashrc

################################
# TROUBLESHOOTING ########
################################# 

# To start the simulation : ros2 launch turtlebot3_gazebo_custom turtlebot3_world.launch.py
# To start the webserver : ros2 launch rosbridge_server rosbridge_websocket_launch.xml 
# To start a cloudflare ssl tunnel : cloudflared tunnel run laboratory

# To start a (dev) secure webserver : ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 ssl:=true certfile:=/mnt/c/Users/nerbe/Development/kuroxy-personal-website/certificates/cert.pem keyfile:=/mnt/c/Users/nerbe/Development/kuroxy-personal-website/certificates/key.pem authenticate:=false

# To generate a temporary certificate : openssl req -x509 -nodes -days 365 -newkey rsa:2048   -keyout key.pem -out cert.pem   -subj "/CN=192.168.1.16"   -addext "subjectAltName=IP:192.168.1.16"

# To setup the cloudflare tunnel : cloudflared tunnel create laboratory

# Then, configure it:
# sudo nano config.yaml (in the .cloudflared folder).
# tunnel: laboratory 
# credentials-file: /home/nicolas/.cloudflared............ 
# ingress: 
#   - hostname: laboratory.nico-kuroro.xyz 
#     service: http://localhost:9090     
#   - service: http_status:404

# Finally, set up the DNS in cloudflared: 
# Create a CNAME, put the same tunnel name, and set the target as the name of your credential followed by .cfargotunnel.com). 
# This will create laboratory.nico-kuroro.xyz as a proxy for the website... 

# Don't forget to set the NEXT_PUBLIC_ROS_IP in .env ! "laboratory.nico-kuroro.xyz"

# Doc from : 
# https://developers.cloudflare.com/cloudflare-one/connections/connect-networks/do-more-with-tunnels/local-management/create-local-tunnel/
# https://developers.cloudflare.com/cloudflare-one/connections/connect-networks/routing-to-tunnel/dns/ 
