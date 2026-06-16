#!/bin/bash
# ---------------------------------------------------------------------------
# rebuild.sh — rebuild library and ROS 2 node after any code change
# Run this after git pull to apply changes
# Usage: ~/gr0x-motor/rebuild.sh
# ---------------------------------------------------------------------------
set -e

echo "=== Rebuilding motor library ==="
cd ~/gr0x-motor/build
make -j4

echo ""
echo "=== Rebuilding ROS 2 node ==="
source /opt/ros/jazzy/setup.bash
cd ~/gr0x-ws
colcon build

echo ""
echo "=== Restarting motor service ==="
sudo systemctl restart gr0x-motor.service
sleep 2
sudo systemctl status gr0x-motor.service | grep Active

echo ""
echo "=== Done ==="
