#!/bin/bash
#sudo modprobe slcan
#sudo modprobe can
#sudo modprobe can_raw

#sudo slcand -o -c -s8 /dev/ttyACM1 can0
#sudo ip link set can0 up

colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash

# roboware_node
#gnome-terminal --tab --title="roboware_node" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run harurobo_pkg roboware_node; exec bash"

# ps3
gnome-terminal --tab --title="ps3" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run harurobo_pkg ps3; exec bash"

# can_nodeを新しいタブで開始
gnome-terminal --tab --title="can_node" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run harurobo_pkg can_node; exec bash"
