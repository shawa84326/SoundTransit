

# ROS Web Interface

This project connects to a ROS (Robot Operating System) network via WebSockets and allows users to control robots remotely. The web interface provides a live feed from a camera and buttons for controlling the robot.

## Prerequisites

Before running the project, make sure you have the following installed:

1. **Node.js**: This project requires Node.js for running the React application.
   - [Download Node.js](https://nodejs.org/)

2. **ROS (Robot Operating System)**: You need to have ROS set up and running, and the `rosbridge_websocket` package installed to enable communication between the web interface and the robot.
   - [ROS Installation Guide](http://wiki.ros.org/ROS/Installation)
   - Install `rosbridge_websocket`:
     ```bash
     sudo apt-get install ros-noetic-rosbridge-server
     ```

3. **Web Video Server**: The live feed is fetched via a stream from the `web_video_server` in ROS. Set it up and stream images from your camera device.
   - [Web Video Server Setup](http://wiki.ros.org/web_video_server)

## Getting Started

1. **Clone the repository**:
   ```bash
   git clone https://github.com/yourusername/ros-web-interface.git
   cd ros-web-interface
   ```

2. **Install dependencies**:
   Run the following command to install the required npm packages:
   ```bash
   npm install
   ```

3. **Configure ROS Connection**:
   Open the `src` folder in your project and edit the WebSocket URL inside `src/Page.js` to match your ROSBridge WebSocket connection.
   
   ```js
   const rosInstance = new ROSLIB.Ros({
     url: 'ws://<YOUR_ROS_IP>:9090',  // Replace with your ROSBridge server IP
   });
   ```

4. **Run the application**:
   Start the development server:
   ```bash
   npm start
   ```
   This will start the React app, and it will automatically open in your browser at `http://localhost:3000`.

## Usage

Once the app is running, you'll see:

- A live feed from your camera at the top of the page.
- Control buttons for sending commands to the robot below the camera feed.
- A table to monitor waste types, status, and robot location.
- Buttons to control specific robot tasks like "Suction", "Scrubber", and "To Dock".
- Progress bars will pop up when performing tasks, showing completion status.

## Notes

- The camera feed URL (`IMAGE_URL`) is fetched from a ROS web video server. Ensure your ROS environment is correctly set up for streaming.
- WebSocket server (`ws://<YOUR_ROS_IP>:9090`) must be running for communication between the web interface and ROS.

## Troubleshooting

- If the live feed is not displaying, ensure that the Web Video Server in ROS is running and streaming from the correct topic (`/output_image`).
- If you encounter issues with the ROS connection, ensure that the IP address of the ROSBridge WebSocket server is correct and that the server is accessible.

---

### License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

You can update the repository link and any project-specific configurations as needed.
