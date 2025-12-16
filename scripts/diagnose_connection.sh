#!/bin/bash
# Mid360 Connection Diagnostic Script

echo "========================================"
echo "Mid360 Connection Diagnostics"
echo "========================================"
echo ""

INTERFACE="eth0"

echo "1. Checking Ethernet interface status..."
echo "----------------------------------------"
ip link show $INTERFACE
echo ""

echo "2. Checking carrier status..."
echo "----------------------------------------"
CARRIER=$(cat /sys/class/net/$INTERFACE/carrier 2>/dev/null || echo "error")
echo "Carrier: $CARRIER (1=connected, 0=no cable)"
echo ""

echo "3. Checking link with ethtool..."
echo "----------------------------------------"
if command -v ethtool &> /dev/null; then
    sudo ethtool $INTERFACE | grep -E "Link detected|Speed|Auto-negotiation"
else
    echo "ethtool not installed"
fi
echo ""

echo "4. Checking for any DHCP activity..."
echo "----------------------------------------"
timeout 5 sudo tcpdump -i $INTERFACE -c 5 2>&1 | head -10 &
TCPDUMP_PID=$!
sleep 5
echo ""

echo "5. System log for Ethernet events..."
echo "----------------------------------------"
sudo dmesg | grep -i eth0 | tail -5
echo ""

echo "========================================"
echo "Diagnostic Summary:"
echo "========================================"
if [ "$CARRIER" == "1" ]; then
    echo "✓ Cable is physically connected"
    echo "  Next: Configure IP and test connectivity"
else
    echo "✗ No carrier detected"
    echo ""
    echo "Checklist:"
    echo "  [ ] Is Mid360 powered on?"
    echo "  [ ] Do you see LEDs on the Mid360?"
    echo "  [ ] Is Ethernet cable firmly connected on BOTH ends?"
    echo "  [ ] Is cable plugged into Mid360's Ethernet port (not USB)?"
    echo "  [ ] Try a different Ethernet cable"
    echo "  [ ] Wait 30 seconds after power-on for Mid360 to boot"
fi
echo ""
