#!/bin/bash



# Open a new terminal tab and run roslaunch for pi_launch
gnome-terminal --tab -- bash -c "roslaunch create_navigation laptop_launch.launch; exec bash" &

# Wait for 5 seconds
sleep 5


