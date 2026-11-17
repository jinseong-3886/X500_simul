#!/bin/bash
# Add a MAVLink 'onboard' instance in SITL that talks to the Jetson (192.168.0.7:14540).
cd ~/PX4-Autopilot/build/px4_sitl_default || exit 1
pgrep -f "bin/px4" >/dev/null || { echo "SITL not running"; exit 1; }
ip -4 addr show eth0 2>/dev/null | grep inet
ping -c1 -W2 192.168.0.7 >/dev/null && echo "WSL -> Jetson ping OK" || echo "WSL -> Jetson ping FAILED (may be blocked; UDP may still work)"
./bin/px4-mavlink start -u 14600 -r 400000 -m onboard -o 14540 -t 192.168.0.7
sleep 1
./bin/px4-mavlink status 2>&1 | grep -E "instance|partner|mode|udp" | head -30
