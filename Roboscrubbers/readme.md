
# Elevator Cleaning Robot Project

## Overview
This project focuses on developing an autonomous elevator cleaning robot designed to clean elevators efficiently in multi-floor buildings. The primary application is for the Sound Transit project, where the robot is capable of navigating through elevator spaces and performing cleaning tasks autonomously.

## Prerequisites
Before executing the following command, please ensure the following software and dependencies are installed and set up on your system:

- **ROS1** (Robot Operating System 1)
- **Linux (Ubuntu 20.04)**

## Setup Instructions

### 1. Clone the Repository
Clone this repository to your local machine if you haven't already:

```bash
git clone https://github.com/shawa84326/SoundTransit.git
```

### 2. Navigate to the Project Directory
After cloning the repository, navigate into the `Roboscrubbers` folder:

```bash
cd SoundTransit/Roboscrubbers
```

### 3. Give Permission to the Script
If the script `connect_to_client.sh` doesn't have execution permissions, run the following command to grant the necessary permissions:

```bash
chmod +x connect_to_client.sh
```

### 4. Execute the Command
After providing the correct permissions, execute the following command on JETSON/RASPERRY PI to connect the elevator cleaning robot to the client (Laptop):

```bash
./connect_to_client.sh
```

If you encounter a "permission denied" error, ensure the script has execution permissions by running the `chmod +x` command above and try again.

## Troubleshooting
If you face any issues while executing the script, follow these steps:

1. **Ensure ROS1 and Ubuntu 20.04 are correctly installed.**
   - Verify your ROS1 installation by running:
     ```bash
     roscore
     ```
   - If ROS1 is not installed, follow the [ROS installation guide](http://wiki.ros.org/ROS/Installation).

2. **Check Environment Setup**
   - Ensure your ROS environment is properly sourced. Add the following to your `.bashrc` file:
     ```bash
     source /opt/ros/noetic/setup.bash
     ```
   - Reload the `.bashrc` file:
     ```bash
     source ~/.bashrc
     ```

3. **Verify Script Permissions**
   - If you encounter a "permission denied" error, ensure the script has execution permissions:
     ```bash
     chmod +x connect_to_client.sh
     ```

4. **Debugging the Script**
   - If the script fails to execute, check for errors by running:
     ```bash
     bash -x connect_to_client.sh
     ```
   - This will provide detailed output to help identify the issue.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments
This project is part of the Capstone project sponsored by Sound Transit initiative, made at GIX, university of washington and aims to improve cleaning efficiency in public transportation facilities, specifically in elevators across multi-floor buildings.
