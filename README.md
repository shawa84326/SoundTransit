

# Elevator Cleaning Robot

This project focuses on developing an autonomous robot that can clean elevators. The robot interacts with elevator floors, navigates through obstacles through teleoperation, and performs cleaning tasks like suction and scrubbing. It uses a combination of autonomous navigation, sensors, and AI-powered controls to ensure efficient and reliable operation. It depicts the user-journey for removing bio-waste from the elevator

## Project Overview

The **Elevator Cleaning Robot** is designed to be a mobile robot capable of:

- 
- **Task Execution**: Performing cleaning tasks such as scrubbing floors and suctioning dirt.
- **Elevator Interaction**: Entering and exiting elevators through teleop, ensuring that the cleaning process is robust and faster.

## Hardware Requirements

The robot utilizes:

- **Jetson Orion AGX / Raspberry Pi**: For processing and controlling the robot's tasks and navigation.
- **Cameras**: For object recognition and pathfinding.
- **Motors and Wheels**: For mobility and precise control.
- **Cleaning Mechanisms**: Suction and scrubbing tools integrated on the robot.

## Software Requirements

1. **Robot Operating System (ROS)**: The robot runs on ROS for handling communication, sensors, and navigation.
2. **Python 3.x**: Required for scripting and controlling the robot.
3. **OpenCV**: For image processing and camera integration.
4. **Robot Localization Libraries**: For precise location tracking and mapping.
5. **web_video_server**: For streaming video from the robot's cameras to the web interface.

## Setting Up

### Step 1: Clone the Repository

Clone the project repository to your local machine (laptop/PC) and the Jetson/Raspberry Pi:

```bash
git clone https://github.com/shawa84326/SoundTransit.git
cd Roboscrubbers
```

### Step 2: Install Dependencies

Make sure that all necessary libraries and dependencies are installed.

#### On Laptop:

1. Install **ROS Noetic** (or another version compatible with your system).
2. Install **Python** and **OpenCV**:
   ```bash
   sudo apt-get update
   sudo apt-get install python3-opencv
   ```

#### On Jetson/Raspberry Pi:

1. Install **ROS Noetic** (or other compatible versions based on your environment).
2. Install dependencies for the robot:
   ```bash
   sudo apt-get update
   sudo apt-get install python3-opencv
   ```

3. Install **web_video_server**:
   ```bash
   sudo apt-get install ros-noetic-web-video-server
   ```

4. Ensure that the `web_video_server` package is correctly installed and available in your ROS workspace. This package will allow you to stream the robot's camera feed to a web browser.

### Step 3: ROSBridge WebSocket Setup

Make sure to install and configure **rosbridge_websocket** for communication between the laptop and the Jetson/Raspberry Pi.

On your ROS system, run:

```bash
sudo apt-get install ros-noetic-rosbridge-server
```

Start the rosbridge server:

```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

### Step 4: Execute the Commands

#### On Laptop (Server):

On your laptop, run the following command to start the server that will communicate with the Jetson/Raspberry Pi (client):

```bash
.\connect_to_host.sh
```

#### On Jetson/Raspberry Pi (Client):

On your Jetson or Raspberry Pi, run the following command to connect to the server and start receiving commands:

```bash
.\connect_to_client.sh
```

This will establish communication between the laptop (host) and the Jetson/Raspberry Pi (client), allowing you to send control commands to the robot.

### Step 5: Streaming Video from the Robot

Once everything is set up, you can start streaming the video feed from the robot’s camera to a web interface. Here’s how to do it:

1. On the Jetson or Raspberry Pi, run the following command to start the **web_video_server**:

   ```bash
   rosrun web_video_server web_video_server
   ```

2. Now, you can view the live video stream by navigating to:

   ```
   http://<Jetson_or_RaspberryPi_IP>:8080/stream?topic=/camera/image_raw
   ```

   Replace `<Jetson_or_RaspberryPi_IP>` with the actual IP address of your Jetson or Raspberry Pi. This will open the live video feed in your web browser.

## Robot Control

Once the communication is set up, you can use your laptop to:

- **Navigate the Robot**: Control the robot’s movements using the web interface or terminal commands.
- **Perform Cleaning Tasks**: Start and stop cleaning tasks like suction and scrubbing.
- **Monitor Robot Status**: Get real-time updates about the robot's task status, battery level, and more.

### Key Commands:

- **Start Cleaning**: Begin the cleaning task.
- **Stop Cleaning**: Halt the current cleaning operation.
- **Move Forward**: Move the robot forward by a set distance.
- **Turn Left/Right**: Change the robot’s orientation.
- **Docking**: Return the robot to its docking station.

## Troubleshooting

If you encounter any issues during setup or operation:

1. **Check Network Connection**: Ensure that both the laptop and the Jetson/Raspberry Pi are on the same network.
2. **Verify ROSBridge Connection**: Make sure that `rosbridge_websocket` is running and properly connected.
3. **Check Robot Sensors**: Ensure all sensors are working correctly and the robot is able to detect obstacles.
4. **Web Video Streaming**: If the video feed isn’t working, check the `web_video_server` logs for any errors and ensure the camera is correctly publishing to `/camera/image_raw`.

## Notes

- Make sure the elevator environment is safe for the robot, and obstacles are properly avoided.
- The robot’s cleaning mechanism should be regularly maintained to ensure proper suction and scrubbing functionality.
- Ensure that the elevator is correctly configured for autonomous interaction.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

