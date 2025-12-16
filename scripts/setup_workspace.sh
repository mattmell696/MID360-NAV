#!/bin/bash
# Setup Catkin Workspace for Mid360 Navigation Project
# Run after ROS is installed

set -e  # Exit on error

echo "================================"
echo "Catkin Workspace Setup"
echo "================================"

# Check if ROS is sourced, if not try to source it
if [ -z "$ROS_DISTRO" ]; then
    echo "ROS not sourced in current shell, attempting to source..."
    if [ -f /opt/ros/noetic/setup.bash ]; then
        source /opt/ros/noetic/setup.bash
        echo "ROS Noetic sourced successfully"
    else
        echo "Error: ROS not found. Please install ROS first."
        echo "Run: sudo bash install_ros_noetic.sh"
        exit 1
    fi
fi

echo "ROS Distribution: $ROS_DISTRO"
echo ""

# Create workspace
WORKSPACE_DIR="$HOME/catkin_ws"
echo "Creating catkin workspace at: $WORKSPACE_DIR"

mkdir -p "$WORKSPACE_DIR/src"
cd "$WORKSPACE_DIR"

# Initialize workspace
echo "Initializing catkin workspace..."
catkin_make

# Source workspace
echo "Sourcing workspace..."
source "$WORKSPACE_DIR/devel/setup.bash"

# Add to bashrc if not already there
if ! grep -q "source $WORKSPACE_DIR/devel/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# Source Mid360 catkin workspace" >> ~/.bashrc
    echo "source $WORKSPACE_DIR/devel/setup.bash" >> ~/.bashrc
    echo "Added workspace sourcing to ~/.bashrc"
fi

echo ""
echo "================================"
echo "Installing ROS dependencies..."
echo "================================"

sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-tf \
    ros-$ROS_DISTRO-message-filters \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-pcl-conversions \
    libpcl-dev \
    libeigen3-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    cmake \
    git

echo ""
echo "================================"
echo "Workspace setup complete!"
echo "================================"
echo ""
echo "Workspace location: $WORKSPACE_DIR"
echo "Please run: source ~/.bashrc"
echo "Or open a new terminal to activate the workspace"
echo ""
