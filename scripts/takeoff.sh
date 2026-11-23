#!/bin/bash

set -e

echo "=== Setting GUIDED mode ==="
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
  "{base_mode: 0, custom_mode: GUIDED}"

echo "=== Arming ==="
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
  "{value: True}"

echo "=== Taking off to 2m ==="
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL \
  "{min_pitch: 0, yaw: 0, altitude: 2}"

echo "=== Sequence finished ==="
