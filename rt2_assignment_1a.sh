#!/bin/bash

gnome-terminal --tab --title="roscore" -- bash -c "source ros.sh; roscore"
gnome-terminal --tab --title="user_interface" -- bash -c "sleep 1; source ros.sh; roslaunch rt2_assignment1 sim_action.launch"
gnome-terminal --tab --title="position_server" -- bash -c "source ros.sh sleep 2; rosrun rt2_assignment1 go_to_point_action.py"
