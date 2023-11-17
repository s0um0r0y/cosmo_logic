# eYRC --- e-Yantra Robotics Competition 2023 --- Cosmo Logistic (CL). #
![figure0](https://portal.e-yantra.org/img/theme/cl.png)
## INTRODUCTION ##

e-Yantra is a robotics outreach program funded by the Ministry of Education and hosted at IIT Bombay. The goal is to harness the talent of young engineers to solve problems using technology across a variety of domains such as: agriculture, manufacturing, defence, home, smart-city maintenance and service industries.

## THEME INTRODUCTION ##

The “Cosmo Logistic” theme of eYRC 2023-24 is set in a warehouse used for inter-planet logistics from a space station. A robotic arm and mobile robot collaborate to sort and prepare packages to be transported to different planets. In this theme, our team  developed an algorithm for sorting packages autonomously with the help of a robotic arm and mobile robot shown in the figure1.

![figure1](https://github.com/s0um0r0y/cosmo_logic/blob/main/img1.png)

We have learnt to navigate this mobile robot with the help of SLAM (simultaneous localization and mapping) method in a warehouse. We have detected and localised the packages placed on racks, and manipulated the robotic arm to pick them up. In stage 1 of the competition, our team  completed this theme in a simulator (Gazebo).

# Project Overview

## Learnings
- Robot Operating System 2 (ROS 2)
- Gazebo
- MoveIt 2
- Computer Vision
- Git
- RViz 2
- Nav 2

## Implementation
Simulator + Real Industrial Robot (Remote Access)

## System Requirements
- **Operating System:** Ubuntu 22.04 LTS
- **Processor:** Eight cores, x86_64 (64-bit x86 Instruction Set)
- **Storage:** HDD or SSD with at least 50GB of space
- **RAM:** 8GB or more
- **Graphics Card:** Dedicated graphics card preferred
- **Internet:** Minimum speed of 5 Mbps & latency <= 50ms

# Competition Timeline

## 1st to 25th Aug, 2023 - Registrations

Team of two to four students from the same college, any year/discipline can register for the competition.

## 1st Week of Sept, 2023 - Theme Assignment

Based on the members’ filled profile details and theme preferences, e-Yantra will assign the best-suited theme for the team.

After the registration and payment, teams will be inducted into Stage 1 of the competition.

## Sept, 2023 - Nov, 2023 - Stage 1

Stage 1 is divided into three tasks: Task 0, Task 1, and Task 2. All the tasks will be either in simulation or will be software-based (MOOC).

Top-performing teams will be selected for Stage 2 based on their Stage 1 performance.

# Learnings and Tasks

| #   | Task              | Start Date | End Date   | Duration  |
| --- | ----------------- | ---------- | ---------- | --------- | 
| 1   | Task 0            | 05 Sep 2023 | 18 Sep 2023 | 2 Weeks   | 
| 2   | Coding Contest    | 05 Sep 2023 | 01 Oct 2023 | ~4 Weeks  | 
| 3   | Task 1A           | 19 Sep 2023 | 16 Oct 2023 | ~4 Weeks  | 
| 4   | Task 1B           | 19 Sep 2023 | 16 Oct 2023 | ~4 Weeks  | 
| 5   | Task 1C           | 19 Sep 2023 | 16 Oct 2023 | ~4 Weeks  |
| 6   | Task 2A           | 17 Oct 2023 | 10 Nov 2023 | ~4 Weeks  | 
| 7   | Task 2B           | 17 Oct 2023 | 10 Nov 2023 | ~4 Weeks  | 

# Task 0: Python Coding Contest

In this task, we have applied the primary usage of the Python language. This task is a team contest hosted on CodeChef. We have solved  a list of 10 problems. A total of 1475 teams registered for this coding contest:

| # | Problem Description                                  | Code        | Points | Success Rate |
|---|------------------------------------------------------|-------------|--------|--------------|
| 1 | Palindrome                                           | PAL_PY      | 1380   | 58.78%       |
| 2 | IFs and FORs with Python                              | IFFOR1_PY   | 1352   | 60.53%       |
| 3 | Arithmetic Progression with Lambda function          | APLAM1_PY   | 1345   | 68.28%       |
| 4 | Stars in our pattern                                  | STAR_PY     | 1340   | 65.76%       |
| 5 | Count the Characters in WORDS                         | WLEN_PY     | 1337   | 63.92%       |
| 6 | Decimal to Binary Converter                           | D2BIN1_PY   | 1329   | 65.6%        |
| 7 | Calculate the distance between 2 points               | DIST1_PY    | 1328   | 43.3%        |
| 8 | Find the Toppers                                      | SCOR_PY     | 1312   | 43.77%       |
| 9 | Manage the Inventory                                   | INV_PY      | 1279   | 35.42%       |
| 10 |   Slice the list                                 |   SLICE1_PY   |  1220     |  32.89      |


# Task 1: Theme Setup - Introduction to ROS 2

The aim of this task is to complete installation, learn about ROS 2, and set up the robot environment for this theme.

This task is divided into three parts:

## 1. Installation
- Install Ubuntu 22.04 (Jammy Jellyfish) and ROS 2 (Humble).

## 2. Learning Resources
- Explore the following learning resources to get hands-on experience with ROS 2 and Python.

## 3. Warehouse Setup
- Install and build the required packages for this theme.
- Start exploring the robot environment in Gazebo.

# Warehouse Setup Instructions
In this task, your mission is not to construct a robot from the ground up. Instead, you will be provided with a mobile robot model. Your goal is to perform the robot unboxing ceremony and prepare it for subsequent tasks. Proceed to the task instructions to set up your workspace and ready your robot for its grand debut in the interstellar realm.

1. Problem Statement:
The task's objective is to configure the simulation of the warehouse world for the Cosmo Logistic theme on your system. Ensure that the world is properly set up with the required packages.

2. Procedure:
**Build Packages:**
Begin by creating a workspace; refer to the link for instructions on how to do so. Once completed, compile and source the packages.

**Clone the Cosmo Logistic (CL) Repository:**
Navigate to your colcon_ws directory and clone the Cosmo Logistic theme repository (Ensure the src folder is empty):

```bash
cd ~/colcon_ws
git clone https://github.com/eYantra-Robotics-Competition/eYRC-2023_Cosmo_Logistic ./src/
git checkout tags/v1.0.1 .
```

Note: After task 0, to return to the remaining tasks, use `git checkout main`. For users without git installed, use the following command for installation:

```bash
sudo apt install git
```

The Cosmo Logistic package may take some time to clone, depending on your internet speed.

**Install Additional Packages:**
Before building the workspace and applying gazebo changes, use the following commands:

```bash
cd ~/colcon_ws/src/  # assuming your workspace is named as colcon_ws
. requirements.sh
```

To source the installed gazebo file:

```bash
echo "source /usr/share/gazebo-11/setup.bash" >> ~/.bashrc
source ~/.bashrc      # source bashrc as we have made changes
```

Navigate to the colcon_ws directory and build the colcon workspace using the colcon build command.

Note: To build the package in the system, ensure that the terminal is pointing at the ~/colcon_ws directory and not in ~/colcon_ws/src.

```bash
cd ~/colcon_ws
colcon build
```

The setup is now complete!

After the package has been successfully built, do not forget to source it:

```bash
source install/setup.bash
```

For every new package cloned or created within the src of your colcon workspace, build and source the workspace to proceed.

To avoid sourcing the setup file every time, add this to your bashrc using the following command:

```bash
echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc      # source bashrc as we have made changes
```

To run the task 0 launch file, enter:

```bash
ros2 launch ebot_description ebot_gazebo_launch.py
```

This should open the Gazebo application with the mobile robot (named as ebot) spawned inside a warehouse.
![figure2](https://github.com/s0um0r0y/cosmo_logic/blob/main/task_0_gazebo.png)

*This repository contains three packages (as of now):*

- `aws-robomaker-small-warehouse-world`: Contains warehouse rack and package models
- `ebot_description`: Contains mobile robot (ebot) description model
- `eyantra_warehouse`: Contains the warehouse world model

Certainly! Here's the structured content for your GitHub README file:

```markdown
# Task 1: Repository Update and Setup

Before proceeding with Task 1, make sure to update your repository. If you have already cloned the eYRC - Cosmo Logistics Repo during Task 0, follow these steps. If not, you can directly clone the repo using:

```bash
git clone https://github.com/eYantra-Robotics-Competition/eYRC-2023_Cosmo_Logistic.git
```

For those who have made changes and want to preserve them:

1. Navigate inside the `src` folder of your workspace.
2. Stash your changes using the following command:

```bash
git stash
```

3. Pull the updated Task 1 commits from the repo:

```bash
git pull
git checkout tags/v1.1.1 . # if doesn't work try `git fetch` and try again
```

4. Pop your changes from stash:

```bash
git stash pop
```

Note:

- For those who haven't changed any files, you can directly run `git pull` and continue.
- For directly starting with Task 1A.

Next, install required ROS packages for Task 1A:

```bash
sudo apt-get install ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller ros-humble-controller-manager
```

Do a `colcon build` to build the workspace with the updated packages.

**Note:** After any changes in any files, to reflect the same on the workspace, you need to do `colcon build`.

If you want to skip the step of building every time a file (python script) is changed (note: this doesn't include the addition of files or folders), you may use the `colcon build --symlink-install` parameter with the build command. (Surf over the internet to find more about it)

---

## Task 1: Object Pose Estimation, Arm Manipulation, Autonomous Navigation

This task is divided into three parts:

### 1A: Object Pose Estimation
Locate Aruco in the world with respect to the robotic arm.

### 1B: Arm Manipulation using Moveit
Manipulate the robotic arm using the Moveit framework.

### 1C: Autonomous Navigation using Nav2
Navigate eBot in the Gazebo environment using Nav2.

# Learning Resources for Task 1: Computer Vision with OpenCV

## 1. Computer Vision


# Learning Resources for Task 1: Computer Vision with OpenCV

## 1. Computer Vision

Welcome to the learning resources for Task 1! In this section, we'll guide you through the essentials of computer vision using the OpenCV library.

### Resources:

1. **Official OpenCV Tutorials:**
   - [OpenCV Python Tutorials](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)

2. **ROS cv_bridge Tutorial:**
   - [Converting Between ROS Images And OpenCV Images in Python](https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython)

3. **Aruco Marker Detection Tutorial:**
   - [Aruco Marker Detection with OpenCV](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)

### Installation:

To get started, make sure to install the necessary libraries:

- **Install "pip3":**
  ```bash
  sudo apt install python3-pip
  ```

- **Install "OpenCV2 Python and Numpy":**
  ```bash
  pip3 install opencv-contrib-python==4.7.0.72
  pip3 install numpy==1.21.5
  ```

### Understanding Aruco Markers:

An Aruco marker, when viewed in an image, is a quadrilateral shape defined by some set of pixels. The properties of the shape, such as corners, area, length of sides, diagonals, medians, and bisectors, can be utilized for various applications.

### Application:

The idea is to leverage OpenCV to discover properties of the Aruco marker's position and orientation. Apply your logic to determine the pose of the Aruco marker, enabling the publication of transforms between the Aruco object and the base_link of the robot arm.



# ROS Manipulation Setup

*Recommended time for completing the setup: 20-24 hrs*

In this section, we will learn how to use ROS with the MoveIt2! package to control Robotic Manipulators in the Gazebo Simulator. The skills you acquire here can be applied to control real Robotic Manipulators. The code you write will be translated into actions for actual robots.

## Installation of MoveIt and Related Packages

**Note:** Links for each topic are provided as hyperlinks for detailed reference.

1. **MoveIt Package**
   - [MoveIt Documentation](https://moveit.ros.org/documentation/)

   Install all MoveIt packages, which are binary-built and ready for direct deployment:

   ```bash
   sudo apt install ros-humble-moveit
   ```

2. **Joint State Broadcaster Package**
   - [Joint State Broadcaster Documentation](http://wiki.ros.org/joint_state_publisher)

   This package helps publish joint states to a topic, allowing other dependent nodes to know the state of each joint.

   ```bash
   sudo apt install ros-humble-joint-state-publisher
   ```

3. **Joint Trajectory Controller Package**
   - [Joint Trajectory Controller Documentation](http://wiki.ros.org/joint_trajectory_controller)

   A controller that sends trajectory messages to the robotic arm, specifying joint angle values, velocity, and effort variables.

   ```bash
   sudo apt install ros-humble-joint-trajectory-controller
   ```

4. **MoveIt Servo Package**
   - [MoveIt Servo Documentation](https://moveit.ros.org/)

   This package helps servo the arm using linear velocity input format of a link or actuating all joint angles.

   ```bash
   sudo apt install ros-humble-moveit-servo
   ```

5. **Trimesh Package**
   - [Trimesh Documentation](https://trimsh.org/)

   A Python package for importing mesh files in RViz, enabling the arm to understand environmental collision in its planning scene.

   ```bash
   pip install trimesh
   ```

6. **ROS 2 Control CLI Package**
   - [ROS 2 Control CLI Documentation](https://index.ros.org/p/ros2_control/)

   The ROS 2 control CLI package allows access to each controller in the terminal, similar to accessing ROS 2 topics.

   ```bash
   sudo apt install ros-humble-ros2controlcli
   ```

# MoveIt Setup Assistant for ROS 2



The MoveIt Setup Assistant is a graphical user interface for configuring any robot for use with MoveIt. Its primary function is generating a Semantic Robot Description Format (SRDF) file for your robot, which specifies additional information required by MoveIt such as planning groups, end effectors, and various kinematic parameters. Additionally, it generates other necessary configuration files for use with the MoveIt pipeline. To use the MoveIt Setup Assistant, you will need to have a URDF file for your robot. 

You can skip the steps not shown in the below information. We highly encourage you to explore the internet for more detailed concepts.

1. **Start:**
   (Make sure to source the workspace in the terminal before starting these steps)

   ```bash
   ros2 launch moveit_setup_assistant setup_assistant.launch.py
   ```
   This command will open a window with the GUI of the setup assistant.

2. **Select the URDF file of the robotic arm:**
   - Click on "Create New Moveit Configuration Package."
   - Select the URDF file of the UR5 arm named as `ur5_arm.urdf.xacro` from the `ur_description/urdf` package.
     **Note:** Make sure to select only `ur5_arm.urdf.xacro` and NOT any other xacro file.
   - Click on the "Load Files" button to load the URDF file.

3. **Generate Self-Collision Matrix:**
   (Make sure to keep the bar of sampling density to its maximum value.)
   - Click on the "Self-Collisions" pane selector on the left-hand side.
   - Click on the "Generate Collision Matrix" button. The Setup Assistant will work for a few seconds before presenting you the results of its computation in the main table.

4. **Add Virtual Joints:**
   Virtual joints are used primarily to attach the robot to the world. Add two virtual joints:
   - FixedBase: for `base_link` having parent link as `world`
   - CamBase: for `camera_link` having parent link as `world`

5. **Add Planning Groups:**
   Planning groups are used to semantically describe different parts of your robot.
   - Click on "Add Group" button and provide details to resemble more with hardware.
     - Group Name: `ur_manipulator`
     - Kinematics solver: `KDLKinematicPlugin`
     - Click on "Add Joints" and add the specified joints.
     - Add links as shown.
     - Add a chain by selecting `base_link` as `base_link` and `tool0` as `tool0`.
     The final planning group setup should look like the provided window.

6. **Label End Effectors:**
   - Add an end effector with the following info:
     - EEF Name: `gripper`
     - Select Group Name: `ur_manipulator`
     - Parent Link: `tool0`

7. **Checking ros2_control:**
   - Make sure that ros2_control has position for `command_interface` and position, velocity for `position_interface`.

8. **Add Author Information:**
   - Click on the "Author Information" pane.
   - Enter your name and email address.

9. **Generate Configuration Files:**
   - Click on the "Configuration Files" pane.
   - Choose a location and name for the ROS package that will be generated containing your new set of configuration files.
   - Click on "Generate Package."
   - The Setup Assistant will generate and write a set of launch and config files into the directory of your choosing.

All the generated files will appear in the "Generated Files/Folders" tab, and you can click on each of them for a description of what they contain. Finally, you can exit the setup assistant.



You can use this Markdown code in your README file to guide users through the installation process for ROS Manipulation with MoveIt and related packages.

## TEAM MEMBERS ##
| Team ID | Name                   | Branch | Email ID                                |
|---------|------------------------|--------|-----------------------------------------|
| CL#2277 | Soumo Roy              | ECE    | soumo.roy2021@vitstudent.ac.in           |
| CL#2277 | Joel Viju              | ECE    | joelviju.v2021@vitstudent.ac.in         |
| CL#2277 | Anirudh Singareddy     | CSE    | singareddy.anirudh2021@vitstudent.ac.in |
| CL#2277 | Rohan Raj              | ECE    | rohan.raj2021@vitstudent.ac.in           |






