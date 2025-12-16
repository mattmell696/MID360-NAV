#!/bin/bash
# ROS Noetic Installation Script for Ubuntu 20.04
# Run with: sudo bash install_ros_noetic.sh

set -e  # Exit on error

echo "================================"
echo "ROS Noetic Installation Script"
echo "================================"

# Check Ubuntu version
if [[ $(lsb_release -sc) != "focal" ]]; then
    echo "Warning: This script is for Ubuntu 20.04 (Focal Fossa)"
    echo "Your Ubuntu version: $(lsb_release -sc)"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo "Step 1: Setting up sources.list..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

echo "Step 2: Setting up keys..."
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

echo "Step 3: Updating package index..."
sudo apt update

echo "Step 4: Installing ROS Noetic Desktop Full..."
sudo apt install -y ros-noetic-desktop-full

echo "Step 5: Setting up environment..."
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source /opt/ros/noetic/setup.bash

echo "Step 6: Installing dependencies for building packages..."
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

echo "Step 7: Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

echo ""
echo "================================"
echo "ROS Noetic installed successfully!"
echo "================================"
echo ""
echo "Please run: source ~/.bashrc"
echo "Or open a new terminal to use ROS"
echo ""
