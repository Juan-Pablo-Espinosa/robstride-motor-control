#!/bin/bash
# ---------------------------------------------------------------------------
# install_deps.sh — GR-0X Pi first-time setup
# Run once on a fresh Ubuntu 24.04 LTS image.
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
    tree \
    curl \
    software-properties-common

echo ""
echo "=== Adding ROS 2 Jazzy apt repo ==="
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update

echo ""
echo "=== Pinning libs to ROS 2 Jazzy-compatible versions ==="
sudo apt install -y --allow-downgrades \
    liblz4-1=1.9.4-1build1 \
    libzstd1=1.5.5+dfsg2-2build1 \
    zlib1g=1:1.3.dfsg-3.1ubuntu2
echo "liblz4-1 hold
libzstd1 hold
zlib1g hold" | sudo dpkg --set-selections

echo ""
echo "=== Installing ROS 2 Jazzy base ==="
sudo apt install -y ros-jazzy-ros-base python3-colcon-common-extensions

echo ""
echo "=== Configuring ROS 2 in shell ==="
if ! grep -q "ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "  Added ROS 2 source to ~/.bashrc"
else
    echo "  ROS 2 already in ~/.bashrc, skipping"
fi

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
echo "=== Installing systemd services ==="
sudo cp "$(dirname "$0")/systemd/gr0x-can.service" /etc/systemd/system/
sudo cp "$(dirname "$0")/systemd/gr0x-motor.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable gr0x-can.service gr0x-motor.service
echo "  Services installed and enabled"

echo ""
echo "=== Building ROS 2 node ==="
source /opt/ros/jazzy/setup.bash
mkdir -p ~/gr0x-ws/src
ln -sf ~/gr0x-motor/ros2 ~/gr0x-ws/src/gr0x_motor_node
cd ~/gr0x-ws
colcon build --symlink-install
echo "source ~/gr0x-ws/install/setup.bash" >> ~/.bashrc

echo ""
echo "=== Done! Next steps: ==="
echo "  1. Reboot for HAT overlays: sudo reboot"
echo "  2. After reboot, bring up CAN: sudo ~/gr0x-motor/setup.sh"
echo "  3. Test motors: cd ~/gr0x-motor/build && ./scan_test"
echo "  4. Run ROS node: ros2 run gr0x_motor_node gr0x_motor_node"