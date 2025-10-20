# ROS 2 Tutorial: Your First Robotics Project (The Tom & Jerry Chase)

> **Note from SG-Robotics-Open-Source:** This repository is the official "getting started" tutorial for our community. It is based on an original project by Shahazad Abdulla.

> **Original Author:** [ShahazadAbdulla](https://github.com/ShahazadAbdulla)
> **Original Repository:** [ShahazadAbdulla/ros2-tom-and-jerry](https://github.com/ShahazadAbdulla/ros2-tom-and-jerry)

---

Welcome! This project is designed to be your very first introduction to working with ROS 2 and Git. By completing this tutorial, you will run a fun "Tom & Jerry" chase simulation where one turtle (Tom) chases another (Jerry).

This is a simple, visual way to learn the fundamental workflow of a robotics software project.

<table>
  <tr>
    <td align="center"><strong>Simulation Start</strong></td>
    <td align="center"><strong>Tom Chasing Jerry</strong></td>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/debd05f2-c5d0-43e6-a37f-0f8280ff0d1e" alt="Sim Start" width="300"></td>
    <td><img src="https://github.com/user-attachments/assets/5b998e37-4da7-4622-92cc-699d741e5185" alt="Tom Chasing" width="300"></td>
  </tr>
  <tr>
    <td align="center"><strong>Jerry Runs Away</strong></td>
    <td align="center"><strong>The Chase Continues</strong></td>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/0c67263e-0d17-46bd-a400-6286ca40b172" alt="Jerry Runs" width="300"></td>
    <td><img src="https://github.com/user-attachments/assets/43d46cf6-598e-46f1-9be8-12f40ce70ea7" alt="Another Chase Cycle" width="300"></td>
  </tr>
</table>

---

## What You Will Learn

By the end of this tutorial, you will know how to:
*   **Clone** a Git repository from GitHub.
*   **Build** a ROS 2 package using `colcon`.
*   **Source** your workspace to make ROS 2 aware of your packages.
*   **Run** a complete application using a ROS 2 `launch` file.
*   **Understand** the basic workflow you will use for all other robotics projects.

---

## Step 1: Prerequisites - Setting Up Your Environment

Before you begin, you need three things:

1.  **ROS 2 Humble:** If you do not have ROS 2 installed, please follow the **[Official Ubuntu Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)**.
    ```bash
    # Use this command to add the source script to your bashrc file so you dont have to type the above command everytime you open a terminal(mentioned in the installation page)
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    ```
3.  **TurtleSim Package:** This is the package we will need to run this simulation.
    ```bash
    # Make sure you have this package.
    sudo apt install ros-humble-turtlesim
    ```
4.  **Git:** This is usually pre-installed on Ubuntu.
    
---

## Step 2: The Standard Workflow (Clone, Build, Run)

This entire repository is a ROS 2 workspace. You just need to clone it, build it, and run it.

### 2.1 - Clone the Workspace

Open a terminal in your Home directory (or wherever you like to keep your projects) and clone this repository.
```bash
# This command downloads the entire workspace folder to your computer.
git clone https://github.com/SG-Robotics-Open-Source/ros2_tutorial_ws.git
```

### 2.2 - Build the Package

Now that you have the code, you need to build it so ROS 2 can execute it.

```bash
# Navigate back to the root of your workspace.
# You must always run the build command from here.
cd ~/ros2_tutorial_ws
```

```bash
# Use the ROS 2 build tool, colcon, to build the package we just cloned.
colcon build --packages-select turtle_control
```

### 2.3 - Source and Run the Simulation

Your code is built! The final step is to run it.

```bash 
# 'Sourcing' updates your terminal, letting it know where to find the package you just built.
# IMPORTANT: You must run this command in any new terminal you open.
source install/setup.bash
```
```bash
# Use 'ros2 launch' to find our package and run the main launch file.
# This single command starts the simulation and our code.
ros2 launch turtle_control tom_and_jerry.launch.py
```

A new window should pop up showing a "Tom" turtle chasing a "Jerry" turtle. Congratulations! You have just completed the standard workflow for a ROS 2 project.

## How It Works: A Quick Look Inside

This project is a ROS 2 package that contains:
*   **A Launch File (`launch/`):** A script that automatically starts the TurtleSim simulator and our Python control node.
*   **A Python Node (`turtle_control/`):** A single script that contains all the logic for both Tom and Jerry. It uses ROS 2 Topics to get the turtles' positions and send velocity commands.
*   **Package Files (`package.xml`, `setup.py`):** Configuration files that define the package and tell `colcon` how to build it.

## Congratulations and Next Steps

You've successfully cloned, built, and run a standard ROS 2 project! You are now familiar with the basic skills needed for robotics software development.

You are ready to tackle a more advanced project. We highly recommend you explore our flagship robot simulation:
### **[Project Orion UGV](https://github.com/SG-Robotics-Open-Source/ros2-project-orion-ugv)**

