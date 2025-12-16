#!/bin/bash
# Install Livox SDK2 and livox_ros_driver2 for Mid360 support
# Run with: bash install_livox.sh

set -e  # Exit on error

echo "========================================"
echo "Livox SDK2 and Driver Installation"
echo "========================================"

# Check if ROS is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS..."
    source /opt/ros/noetic/setup.bash
fi

echo "ROS Distribution: $ROS_DISTRO"
echo ""

# Define directories
LIVOX_SDK_DIR="$HOME/Livox-SDK2"
CATKIN_WS="$HOME/catkin_ws"

# =====================================
# Step 1: Install Livox SDK2
# =====================================
echo "Step 1: Installing Livox SDK2..."
echo "------------------------------"

# Remove old SDK2 if exists
if [ -d "$LIVOX_SDK_DIR" ]; then
    echo "Removing existing Livox-SDK2 directory..."
    rm -rf "$LIVOX_SDK_DIR"
fi

# Clone Livox SDK2
echo "Cloning Livox-SDK2 repository..."
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2

# Build and install SDK2
echo "Building Livox-SDK2..."
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

echo "Installing Livox-SDK2 (requires sudo)..."
sudo make install

echo "✓ Livox SDK2 installed successfully"
echo ""

# =====================================
# Step 2: Install livox_ros_driver2
# =====================================
echo "Step 2: Installing livox_ros_driver2..."
echo "------------------------------"

# Go to catkin workspace
cd "$CATKIN_WS/src"

# Remove old driver if exists
if [ -d "livox_ros_driver2" ]; then
    echo "Removing existing livox_ros_driver2 directory..."
    rm -rf livox_ros_driver2
fi

# Clone livox_ros_driver2
echo "Cloning livox_ros_driver2 repository..."
git clone https://github.com/Livox-SDK/livox_ros_driver2.git

# Build the driver
echo "Building livox_ros_driver2..."
cd "$CATKIN_WS"
source devel/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="livox_ros_driver2"

# Reset whitelist
catkin_make -DCATKIN_WHITELIST_PACKAGES=""

echo "✓ livox_ros_driver2 installed successfully"
echo ""

# =====================================
# Step 3: Create Mid360 configuration
# =====================================
echo "Step 3: Creating Mid360 configuration..."
echo "------------------------------"

CONFIG_DIR="$CATKIN_WS/src/livox_ros_driver2/config"

# Create backup of original config if not exists
if [ -f "$CONFIG_DIR/MID360_config.json" ] && [ ! -f "$CONFIG_DIR/MID360_config.json.backup" ]; then
    cp "$CONFIG_DIR/MID360_config.json" "$CONFIG_DIR/MID360_config.json.backup"
    echo "✓ Backed up original MID360_config.json"
fi

echo "✓ Configuration files ready for editing"
echo ""

# =====================================
# Verification
# =====================================
echo "========================================"
echo "Installation Verification"
echo "========================================"

echo "Checking Livox SDK2..."
if [ -d "$LIVOX_SDK_DIR" ]; then
    echo "✓ Livox SDK2 directory exists"
else
    echo "✗ Livox SDK2 directory not found"
fi

echo "Checking livox_ros_driver2..."
if [ -d "$CATKIN_WS/src/livox_ros_driver2" ]; then
    echo "✓ livox_ros_driver2 package exists"
else
    echo "✗ livox_ros_driver2 package not found"
fi

echo "Checking build artifacts..."
if [ -f "$CATKIN_WS/devel/lib/livox_ros_driver2/livox_ros_driver2_node" ]; then
    echo "✓ livox_ros_driver2_node binary exists"
else
    echo "✗ livox_ros_driver2_node binary not found"
fi

echo ""
echo "========================================"
echo "Installation Complete!"
echo "========================================"
echo ""
echo "Next steps:"
echo "1. Configure network for Mid360:"
echo "   - Set static IP on your computer (e.g., 192.168.1.50)"
echo "   - Mid360 default IP is typically 192.168.1.1XX"
echo ""
echo "2. Edit driver configuration:"
echo "   $CONFIG_DIR/MID360_config.json"
echo ""
echo "3. Test the driver:"
echo "   roslaunch livox_ros_driver2 msg_MID360.launch"
echo ""
echo "4. Source workspace:"
echo "   source ~/catkin_ws/devel/setup.bash"
echo ""
