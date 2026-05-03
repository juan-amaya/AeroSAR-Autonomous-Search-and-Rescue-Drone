#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

if [ -f "/home/aerosar/ws/install/setup.bash" ]; then
    source /home/aerosar/ws/install/setup.bash
fi

exec "$@"