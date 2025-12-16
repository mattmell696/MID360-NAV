#!/bin/bash
# Network Setup Script for Livox Mid360
# This script configures the network interface for the Mid360 sensor

set -e

echo "========================================"
echo "Livox Mid360 Network Setup"
echo "========================================"
echo ""

# Configuration
INTERFACE="eth0"
HOST_IP="192.168.1.50"
NETMASK="255.255.255.0"
SENSOR_IP="192.168.1.1"  # Default Mid360 IP (adjust if different)

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run with sudo: sudo bash $0"
    exit 1
fi

echo "Configuring interface: $INTERFACE"
echo "Host IP will be: $HOST_IP"
echo "Expected sensor IP: $SENSOR_IP"
echo ""

# Bring interface up
echo "Step 1: Bringing up $INTERFACE..."
ip link set $INTERFACE up
sleep 2

# Check for carrier
CARRIER=$(cat /sys/class/net/$INTERFACE/carrier 2>/dev/null || echo "0")
if [ "$CARRIER" == "0" ]; then
    echo "⚠️  WARNING: No carrier detected on $INTERFACE"
    echo "   - Check if Ethernet cable is connected"
    echo "   - Verify Mid360 is powered on"
    echo "   - Wait 20 seconds for sensor to boot"
    echo ""
    read -p "Press Enter after checking cable connection..."
    sleep 2
    CARRIER=$(cat /sys/class/net/$INTERFACE/carrier 2>/dev/null || echo "0")
fi

if [ "$CARRIER" == "1" ]; then
    echo "✓ Carrier detected - cable is connected"
else
    echo "✗ Still no carrier - please check physical connection"
    exit 1
fi

# Configure static IP
echo ""
echo "Step 2: Configuring static IP..."
ip addr flush dev $INTERFACE
ip addr add $HOST_IP/24 dev $INTERFACE
ip link set $INTERFACE up

echo "✓ IP configured: $HOST_IP"
echo ""

# Show interface status
echo "Interface status:"
ip addr show $INTERFACE | grep -E "inet |state"
echo ""

# Test connectivity
echo "Step 3: Testing connectivity to sensor..."
echo "Pinging $SENSOR_IP (this may take a moment)..."

if ping -c 3 -W 2 $SENSOR_IP > /dev/null 2>&1; then
    echo "✓ SUCCESS! Mid360 is reachable at $SENSOR_IP"
    echo ""
    echo "Network setup complete!"
    echo "You can now run: roslaunch mid360_nav full_system.launch"
else
    echo "✗ Cannot reach sensor at $SENSOR_IP"
    echo ""
    echo "Troubleshooting:"
    echo "1. Verify sensor IP address (may not be $SENSOR_IP)"
    echo "2. Check Mid360 status LEDs"
    echo "3. Try scanning network: sudo nmap -sn 192.168.1.0/24"
    echo "4. Check sensor config: ~/catkin_ws/src/livox_ros_driver2/config/MID360_config.json"
    exit 1
fi

echo ""
echo "========================================"
echo "Network is ready for Mid360!"
echo "========================================"
