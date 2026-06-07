#!/bin/bash
# ---------------------------------------------------------------------------
# setup.sh — GR-0X Pi bringup
# Run once per boot before any motor control.
# Usage: sudo ./setup.sh
# ---------------------------------------------------------------------------

set -e

CAN_BITRATE=1000000

bring_up() {
    local iface=$1
    echo "Bringing up $iface..."
    ip link set "$iface" down 2>/dev/null || true
    ip link set "$iface" type can bitrate $CAN_BITRATE
    ip link set "$iface" up
    echo "  $iface: $(ip link show $iface | grep -oE 'state [A-Z]+')"
}

if [[ $EUID -ne 0 ]]; then
    echo "ERROR: Run as root: sudo ./setup.sh"
    exit 1
fi

bring_up can0
bring_up can1

echo ""
echo "CAN interfaces ready. Build the library:"
echo "  cd ~/gr0x-motor/build && make -j4"
echo ""
echo "Run tests:"
echo "  ./scan_test"
echo "  ./showcase_single"
