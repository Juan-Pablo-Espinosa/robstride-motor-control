#!/bin/bash
# ---------------------------------------------------------------------------
# install_deps.sh — GR-0X Pi first-time setup
# Run once on a fresh Ubuntu 24.04 image.
# Usage: ./install_deps.sh
# ---------------------------------------------------------------------------
set -e

echo "=== GR-0X Pi: installing system dependencies ==="
sudo apt update
sudo apt install -y \
    cmake \
    gcc \
    g++ \
    make \
    can-utils \
    libncurses-dev \
    git \
    tree

echo ""
echo "=== Configuring PiCAN FD Duo ISO HAT ==="
CONFIG=/boot/firmware/config.txt

if ! grep -q "mcp251xfd" $CONFIG; then
    echo "dtparam=spi=on" | sudo tee -a $CONFIG
    echo "dtoverlay=mcp251xfd,spi0-0,oscillator=40000000,interrupt=25" | sudo tee -a $CONFIG
    echo "dtoverlay=mcp251xfd,spi0-1,oscillator=40000000,interrupt=05" | sudo tee -a $CONFIG
    echo "  HAT overlays added to $CONFIG"
else
    echo "  HAT overlays already present, skipping"
fi

echo ""
echo "=== Configuring passwordless CAN bringup ==="
SUDOERS_FILE=/etc/sudoers.d/gr0x-can
if [ ! -f $SUDOERS_FILE ]; then
    echo "gr0x-pi ALL=(ALL) NOPASSWD: /sbin/ip link set can0 down, /sbin/ip link set can0 type can bitrate 1000000, /sbin/ip link set can0 up, /sbin/ip link set can1 down, /sbin/ip link set can1 type can bitrate 1000000, /sbin/ip link set can1 up" \
        | sudo tee $SUDOERS_FILE
    sudo chmod 440 $SUDOERS_FILE
    echo "  Sudoers configured"
else
    echo "  Sudoers already configured, skipping"
fi

echo ""
echo "=== Building library ==="
cd "$(dirname "$0")"
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

echo ""
echo "=== Done! Next steps: ==="
echo "  1. Reboot for HAT overlays to take effect: sudo reboot"
echo "  2. After reboot, bring up CAN: sudo ~/gr0x-motor/setup.sh"
echo "  3. Test: cd ~/gr0x-motor/build && ./scan_test"
