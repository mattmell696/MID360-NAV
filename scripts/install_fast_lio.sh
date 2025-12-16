#!/bin/bash
# Install FAST-LIO2 and dependencies for Mid360 mapping
# Run with: bash install_fast_lio.sh

set -e  # Exit on error

echo "========================================"
echo "FAST-LIO2 Installation"
echo "========================================"

# Check if ROS is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS..."
    source /opt/ros/noetic/setup.bash
fi

echo "ROS Distribution: $ROS_DISTRO"
echo ""

# Define directories
CATKIN_WS="$HOME/catkin_ws"

# Ensure catkin workspace exists
if [ ! -d "$CATKIN_WS" ]; then
    echo "Error: Catkin workspace not found at $CATKIN_WS"
    echo "Please run setup_workspace.sh first"
    exit 1
fi

cd "$CATKIN_WS/src"

# =====================================
# Step 1: Install ikd-Tree dependency
# =====================================
echo "Step 1: Installing ikd-Tree..."
echo "------------------------------"

if [ -d "ikd-Tree" ]; then
    echo "Removing existing ikd-Tree directory..."
    rm -rf ikd-Tree
fi

echo "Cloning ikd-Tree repository..."
git clone https://github.com/hku-mars/ikd-Tree.git

echo "✓ ikd-Tree cloned successfully"
echo ""

# =====================================
# Step 2: Install FAST-LIO2
# =====================================
echo "Step 2: Installing FAST-LIO2..."
echo "------------------------------"

if [ -d "FAST_LIO" ]; then
    echo "Removing existing FAST_LIO directory..."
    rm -rf FAST_LIO
fi

echo "Cloning FAST-LIO2 repository..."
git clone https://github.com/hku-mars/FAST_LIO.git

echo "✓ FAST-LIO2 cloned successfully"
echo ""

# =====================================
# Step 3: Build workspace
# =====================================
echo "Step 3: Building catkin workspace..."
echo "------------------------------"

cd "$CATKIN_WS"

# Source workspace
source devel/setup.bash

# Build
echo "Building all packages (this may take a few minutes)..."
catkin_make

if [ $? -eq 0 ]; then
    echo "✓ Build completed successfully"
else
    echo "✗ Build failed"
    exit 1
fi

echo ""

# =====================================
# Verification
# =====================================
echo "========================================"
echo "Installation Verification"
echo "========================================"

# Source the workspace for verification
source "$CATKIN_WS/devel/setup.bash"

echo "Checking ikd-Tree..."
if [ -d "$CATKIN_WS/src/ikd-Tree" ]; then
    echo "✓ ikd-Tree package exists"
else
    echo "✗ ikd-Tree package not found"
fi

echo "Checking FAST_LIO..."
if [ -d "$CATKIN_WS/src/FAST_LIO" ]; then
    echo "✓ FAST_LIO package exists"
else
    echo "✗ FAST_LIO package not found"
fi

echo "Checking FAST_LIO binary..."
if [ -f "$CATKIN_WS/devel/lib/fast_lio/fastlio_mapping" ]; then
    echo "✓ fastlio_mapping node exists"
else
    echo "✗ fastlio_mapping node not found"
fi

echo "Checking ROS package recognition..."
if rospack find fast_lio > /dev/null 2>&1; then
    echo "✓ fast_lio package found by ROS"
    echo "  Location: $(rospack find fast_lio)"
else
    echo "✗ fast_lio package not found by ROS"
fi

echo ""
echo "========================================"
echo "Installation Complete!"
echo "========================================"
echo ""
echo "Next steps:"
echo "1. Configure FAST-LIO2 for Mid360 sensor"
echo "2. Create launch files"
echo "3. Test with Mid360 sensor or rosbag"
echo ""
echo "Configuration file location:"
echo "  $CATKIN_WS/src/FAST_LIO/config/"
echo ""
echo "Launch file location:"
echo "  $CATKIN_WS/src/FAST_LIO/launch/"
echo ""
echo "Don't forget to source the workspace:"
echo "  source $CATKIN_WS/devel/setup.bash"
echo ""
