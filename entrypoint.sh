#!/bin/bash
# Gives the GigE camera's network port a link-local address, then starts the
# ROS entrypoint. A camera wired straight to the PC gets no DHCP answer and
# takes a 169.254.x.x address, so the port needs one in the same range or
# Aravis finds no camera. With host networking this sets the host's port.
# Run it again by hand after plugging the camera in: camera-net-setup

if [ -n "$CAMERA_IFACE" ]; then
    if ! ip link show "$CAMERA_IFACE" &>/dev/null; then
        echo "camera-net-setup: no interface $CAMERA_IFACE, skipping" >&2
    elif ip -4 addr show dev "$CAMERA_IFACE" | grep -q inet; then
        echo "camera-net-setup: $CAMERA_IFACE already has an address"
    elif sudo -n ip link set "$CAMERA_IFACE" up \
        && sudo -n ip addr add "${CAMERA_HOST_IP:-169.254.1.1/16}" dev "$CAMERA_IFACE"; then
        echo "camera-net-setup: $CAMERA_IFACE set to ${CAMERA_HOST_IP:-169.254.1.1/16}"
    else
        echo "camera-net-setup: could not set $CAMERA_IFACE" >&2
    fi
fi

# Only the setup when called as camera-net-setup
[ "$(basename "$0")" = camera-net-setup ] && exit 0
exec /ros_entrypoint.sh "$@"
