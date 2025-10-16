#!/bin/bash
# ADS-B Receiver Troubleshooting Script

echo "=== ADS-B Receiver Diagnostics ==="
echo

# Check if services are running
echo "1. Checking if dump1090 is running..."
if systemctl is-active --quiet dump1090-fa; then
    echo "   ✓ dump1090-fa is running"
    echo "   Listening ports:"
    sudo netstat -tlnp | grep dump1090
else
    echo "   ✗ dump1090-fa is NOT running"
    echo "   Starting dump1090-fa..."
    sudo systemctl start dump1090-fa
fi
echo

echo "2. Checking if gpsd is running..."
if systemctl is-active --quiet gpsd; then
    echo "   ✓ gpsd is running"
else
    echo "   ✗ gpsd is NOT running"
    echo "   Starting gpsd..."
    sudo systemctl start gpsd
fi
echo

echo "3. Testing dump1090 connection..."
timeout 5 nc localhost 30003 | head -5
if [ $? -eq 0 ]; then
    echo "   ✓ dump1090 is sending data on port 30003"
else
    echo "   ✗ No data from dump1090 on port 30003"
fi
echo

echo "4. Testing gpsd connection..."
timeout 5 nc localhost 2947 | head -3
if [ $? -eq 0 ]; then
    echo "   ✓ gpsd is responding on port 2947"
else
    echo "   ✗ No data from gpsd on port 2947"
fi
echo

echo "5. Checking network configuration..."
echo "   Your IP address:"
ip addr show | grep 'inet ' | grep -v '127.0.0.1'
echo

echo "   WiFi broadcast address (if applicable):"
ip addr show wlan0 2>/dev/null | grep 'inet ' || echo "   No wlan0 interface"
echo

echo "6. Checking if stratux-aggregator is running..."
if pgrep -f stratux-aggregator > /dev/null; then
    echo "   ✓ stratux-aggregator is running"
    echo "   Process info:"
    ps aux | grep stratux-aggregator | grep -v grep
else
    echo "   ✗ stratux-aggregator is NOT running"
fi
echo

echo "7. Checking GDL90 port (4000)..."
sudo netstat -ulnp | grep :4000
if [ $? -eq 0 ]; then
    echo "   ✓ Something is listening on port 4000"
else
    echo "   ✗ Nothing listening on port 4000"
fi
echo

echo "8. Checking for GDL90 traffic on network..."
echo "   (Listening for 5 seconds...)"
sudo timeout 5 tcpdump -i any -n 'udp port 4000' 2>/dev/null | head -10
echo

echo "=== Diagnostics Complete ==="
echo
echo "Common Issues:"
echo "1. BROADCAST ADDRESS: Your server is hardcoded to 192.168.10.255"
echo "   - Check if your network is 192.168.10.x"
echo "   - If not, you need to change the broadcast address"
echo
echo "2. FIREWALL: Check if firewall is blocking UDP port 4000"
echo "   sudo ufw status"
echo "   sudo ufw allow 4000/udp"
echo
echo "3. WIFI ISOLATION: Some routers have 'client isolation' enabled"
echo "   - This prevents devices from seeing each other"
echo "   - Check your router settings"
echo