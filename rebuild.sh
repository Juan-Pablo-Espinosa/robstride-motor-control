#!/bin/bash
# ---------------------------------------------------------------------------
# rebuild.sh — rebuild everything after any change
# Handles: library, ROS node, joints.yaml, systemd services
# Usage: cd ~/gr0x-motor && ./rebuild.sh
# ---------------------------------------------------------------------------
set -e

echo "=== Pulling latest changes ==="
cd ~/gr0x-motor
git pull

echo ""
echo "=== Rebuilding motor library ==="
cd ~/gr0x-motor/build
make -j4

echo ""
echo "=== Rebuilding ROS 2 nodes ==="
source /opt/ros/jazzy/setup.bash
cd ~/gr0x-ws/build/gr0x_motor_node
cmake ~/gr0x-motor/ros2
make -j4
make install

echo ""
echo "=== Restarting motor service ==="
sudo systemctl restart gr0x-motor.service
sleep 2
sudo systemctl status gr0x-motor.service | grep -E "Active|INFO"

echo ""
echo "=== Done — run walking test with: ==="
echo "  ~/gr0x-ws/install/gr0x_motor_node/lib/gr0x_motor_node/walking_test_node"