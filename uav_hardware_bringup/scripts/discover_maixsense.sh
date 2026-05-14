#!/usr/bin/env bash
# discover_maixsense.sh
#
# Prints a ready-to-paste udev rule snippet for each MaixSense ToF currently
# plugged in. Plug ONE sensor at a time and run this once per sensor to learn
# its serial; tag the corresponding physical direction (front, FL, RL, RR, FR).
#
# Usage:
#   ./discover_maixsense.sh

set -euo pipefail

echo "════════════════════════════════════════════════════════════════"
echo " MaixSense MS-A010 ToF — udev discovery"
echo " (CH340 USB-serial chip: vendor 1a86, product 7523)"
echo "════════════════════════════════════════════════════════════════"
echo ""

found=0
for dev in /dev/ttyUSB* /dev/ttyACM*; do
    [ -e "$dev" ] || continue

    info=$(udevadm info -q property -n "$dev" 2>/dev/null)
    vid=$(echo "$info" | grep "^ID_VENDOR_ID="     | cut -d= -f2)
    pid=$(echo "$info" | grep "^ID_MODEL_ID="      | cut -d= -f2)
    ser=$(echo "$info" | grep "^ID_SERIAL_SHORT="  | cut -d= -f2)
    kpath=$(echo "$info" | grep "^DEVPATH="        | cut -d= -f2)

    # Filter for the CH340 chip (and any clones using same VID/PID)
    if [ "$vid" = "1a86" ] && [ "$pid" = "7523" ]; then
        found=$((found+1))
        echo "─── $dev ───"
        echo "  serial:    ${ser:-<empty — firmware does not expose serial>}"
        echo "  USB path:  $kpath"
        echo ""
        if [ -n "$ser" ]; then
            echo "  Suggested udev rule line (replace <N> with the index 0..4):"
            echo "    SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"1a86\", ATTRS{idProduct}==\"7523\", \\"
            echo "        ATTRS{serial}==\"$ser\", SYMLINK+=\"maixsense_tof_<N>\""
        else
            # Fallback: match on kernel device path (USB hub slot)
            short=$(basename "$kpath")
            echo "  Suggested udev rule line (port-path fallback, no serial):"
            echo "    SUBSYSTEM==\"tty\", KERNELS==\"${short}:*\", SYMLINK+=\"maixsense_tof_<N>\""
        fi
        echo ""
    fi
done

if [ $found -eq 0 ]; then
    echo "No CH340-class USB-serial devices detected."
    echo "Plug a MaixSense into a USB port, then run this script again."
    exit 1
fi

echo "════════════════════════════════════════════════════════════════"
echo " Next steps:"
echo "   1. Plug each sensor IN TURN (one at a time), run this script,"
echo "      and record which physical direction (front, FL, RL, RR, FR)"
echo "      each serial corresponds to."
echo "   2. Edit udev/99-uav-hardware.rules: replace the REPLACE_WITH_*"
echo "      placeholders with the discovered serials."
echo "   3. Install:"
echo "        sudo cp udev/99-uav-hardware.rules /etc/udev/rules.d/"
echo "        sudo udevadm control --reload && sudo udevadm trigger"
echo "   4. Unplug and replug each sensor. Verify:"
echo "        ls -la /dev/maixsense_tof_*"
echo "════════════════════════════════════════════════════════════════"
