#!/bin/bash

# Open a new terminal tab and run roscore
gnome-terminal --tab -- bash -c "roscore; exec bash" &

# Wait for 5 seconds
sleep 5

# Open a new terminal tab and run roslaunch for rosbridge_websocket
gnome-terminal --tab -- bash -c "roslaunch rosbridge_server rosbridge_websocket.launch; exec bash" &

# Wait for 5 seconds
sleep 5

# Open a new terminal tab and run rosrun for web_video_server
gnome-terminal --tab -- bash -c "rosrun web_video_server web_video_server; exec bash" &

# Wait for 5 seconds
sleep 5

# Open a new terminal tab and run roslaunch for pi_launch
gnome-terminal --tab -- bash -c "roslaunch create_navigation pi_launch.launch; exec bash" &

# Wait for 5 seconds
sleep 5

# Open a new terminal tab and run roslaunch for create_2.launch
gnome-terminal --tab -- bash -c "roslaunch create_bringup create_2.launch; exec bash" &

 # Change directory and run the Python script in the correct folder

gnome-terminal --tab -- bash -c "cd YOLOv5_with_RealSense_D455_Depth_Sensing && python3 poop3.py; exec bash" &
