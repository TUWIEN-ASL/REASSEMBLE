#!/bin/bash
# setup_camera.sh

# # Disable autofocus
# v4l2-ctl -d /dev/video0 --set-ctrl=focus_auto=0

# # Set manual focus
# v4l2-ctl -d /dev/video0 --set-ctrl=focus_absolute=10 # Adjust the focus value as needed

# Stop on any error
set -e

# Disable automatic continuous focus
uvcdynctrl --device=/dev/video0 --set='Focus, Automatic Continuous' 0

# Set absolute focus
uvcdynctrl --device=/dev/video0 --set='Focus, Absolute' 1