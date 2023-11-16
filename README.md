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
![figure2](https://portal.e-yantra.org/themeBook/cl/resources/task_0_gazebo.png)

*This repository contains three packages (as of now):*

- `aws-robomaker-small-warehouse-world`: Contains warehouse rack and package models
- `ebot_description`: Contains mobile robot (ebot) description model
- `eyantra_warehouse`: Contains the warehouse world model



## TEAM MEMBERS ##
| Team ID | Name                   | Branch | Email ID                                |
|---------|------------------------|--------|-----------------------------------------|
| CL#2277 | Soumo Roy              | ECE    | soumo.roy2021@vitstudent.ac.in           |
| CL#2277 | Joel Viju              | ECE    | joelviju.v2021@vitstudent.ac.in         |
| CL#2277 | Anirudh Singareddy     | CSE    | singareddy.anirudh2021@vitstudent.ac.in |
| CL#2277 | Rohan Raj              | ECE    | rohan.raj2021@vitstudent.ac.in           |






