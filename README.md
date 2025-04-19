# ROS 2 TurtleSim Tom & Jerry Chase Project

This project implements a simple chase simulation using ROS 2 Humble and TurtleSim.
A 'Tom' turtle chases a 'Jerry' turtle. When Tom gets close, Jerry moves towards a new random location within the boundaries.

## Features

*   Uses separate nodes/logic for Tom (chaser) and Jerry (target).
*   Tom uses Proportional (P) control to calculate velocity towards Jerry.
*   Jerry chooses a random target point within defined boundaries when approached.
*   Includes a ROS 2 Launch file (`tom_and_jerry.launch.py`) to automate the setup:
    *   Starts TurtleSim.
    *   Kills the default turtle.
    *   Spawns 'Tom' and 'Jerry' at specific locations.
    *   Turns Jerry's pen off.
    *   Starts the controller node.

## Prerequisites

*   ROS 2 Humble Hawksbill (or newer)
*   `turtlesim` package (`sudo apt install ros-humble-turtlesim`)
*   Git

## Setup & Build

1.  Clone the repository into your ROS 2 workspace `src` directory:
    ```bash
    cd ~/your_ros2_ws/src
    git clone https://github.com/YOUR_USERNAME/YOUR_REPOSITORY_NAME.git turtle_control # Or desired package name
    ```
2.  Navigate back to the workspace root:
    ```bash
    cd ~/your_ros2_ws
    ```
3.  Install dependencies (if any added later):
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```
4.  Build the package:
    ```bash
    colcon build --packages-select turtle_control # Or your package name
    ```

## Running

1.  Source your workspace setup file:
    ```bash
    source install/setup.bash
    ```
2.  Execute the launch file:
    ```bash
    ros2 launch turtle_control tom_and_jerry.launch.py # Ensure package/launch file name match
    ```

This will start the simulation, configure the turtles, and run the chase controller.

## Code Structure

*   `launch/`: Contains the main launch file.
*   `turtle_control/`: Contains the Python node script (`turtle_tom.py`).
*   `package.xml`, `setup.py`: ROS package definition and build configuration.
*   `.gitignore`: Specifies files ignored by Git.
