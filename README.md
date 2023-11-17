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

![figure3](https://github.com/s0um0r0y/cosmo_logic/blob/main/image3.png)

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

# RViz Interface for MoveIt! Simulation

In the previous module, you completed the MoveIt! setup assistant and successfully generated MoveIt! configuration files. In this module, you will be using those configuration files for the simulation of the robotic arm.

MoveIt! comes with a plugin for the ROS Visualizer (RViz). The plugin allows you to set up scenes in which the robot will work, generate plans, visualize the output, and interact directly with a visualized robot. We will explore the plugin in this tutorial.

## Visualization of Robotic Arm Planning in RViz

Let's start by visualizing the planning of a robotic arm in RViz. Follow these steps:

1. Launch the demo.launch file from the MoveIt configuration package that you created.

    ```bash
    ros2 launch ur5_moveit demo.launch.py
    ```

   This command will launch the robotic arm in RViz.

2. In RViz, you will see a robotic arm and a MotionPlanning display type.

This setup allows you to interact with the simulated robotic arm, visualize planning scenarios, and generate plans using MoveIt! in the RViz interface.

Feel free to explore different functionalities provided by the RViz plugin to enhance your understanding of the robotic arm's behavior in a simulated environment.


You can use this Markdown code in your README file to guide users through the installation process for ROS Manipulation with MoveIt and related packages.


# Gazebo Interface for ROS Manipulation

**Note: Package & file names, and content of config files might be slightly different.**

Now that we have seen how to use Setup assistant and visualize the movement of the arm in Rviz, let's find out how to simulate it in Gazebo. To make the arm move on Gazebo, we need an interface that will take the commands from MoveIt and convey it to the arm in simulation. This interface, in this scenario, is a controller. The controller is basically a generic control loop feedback mechanism, typically a PID controller, to control the output sent to your actuators.

## ROS Controllers Configuration

1. **ROS Controllers Configuration**
   
   A configuration file called `ros_controllers.yaml` has to be created inside the `config` folder of the `ur5_moveit` package. Remove the previous contents (if any) and paste the configuration given below:

   ```yaml
   # ros_controllers.yaml

   controller_manager:
     ros__parameters:
       update_rate: 2000
       joint_trajectory_controller:
         type: joint_trajectory_controller/JointTrajectoryController
       joint_state_broadcaster:
         type: joint_state_broadcaster/JointStateBroadcaster

   joint_trajectory_controller:
     ros__parameters:
       command_interfaces:
         - position
       state_interfaces:
         - position
         - velocity
       joints:
         - elbow_joint
         - shoulder_lift_joint
         - shoulder_pan_joint
         - wrist_1_joint
         - wrist_2_joint
         - wrist_3_joint
       state_publish_rate: 100.0
       action_monitor_rate: 20.0
       allow_partial_joints_goal: false
       constraints:
         stopped_velocity_tolerance: 0.0
         goal_time: 0.0

   joint_state_broadcaster:
     ros__parameters:
       type: joint_state_broadcaster/JointStateBroadcaster
   ```

2. **MoveIt Controllers Configuration**

   A configuration file called `moveit_controllers.yaml` has to be created inside the `config` folder of the `ur5_moveit` package. Remove the previous contents (if any) and paste the configuration given below:

   ```yaml
   # moveit_controllers.yaml

   controller_names:
     - joint_trajectory_controller

   joint_trajectory_controller:
     action_ns: follow_joint_trajectory
     type: FollowJointTrajectory
     default: true
     joints:
       - shoulder_pan_joint
       - shoulder_lift_joint
       - elbow_joint
       - wrist_1_joint
       - wrist_2_joint
       - wrist_3_joint
   ```

3. **OMPL Planning Configuration**

   A configuration file called `ompl_planning.yaml` has to be created inside the `config` folder of the `ur5_moveit` package. Remove the previous contents (if any) and paste the configuration given below:

   ```yaml
   # ompl_planning.yaml

   planning_plugin: 'ompl_interface/OMPLPlanner'
   request_adapters: >-
       default_planner_request_adapters/AddTimeOptimalParameterization
       default_planner_request_adapters/FixWorkspaceBounds
       default_planner_request_adapters/FixStartStateBounds
       default_planner_request_adapters/FixStartStateCollision
       default_planner_request_adapters/FixStartStatePathConstraints
   start_state_max_bounds_error: 0.1
   ```

4. **Servo Information Configuration**

   A configuration file called `ur_servo.yaml` has to be created inside the `config` folder of the `ur5_moveit` package. Remove the previous contents (if any) and paste the configuration given below:

   ```yaml
   # ur_servo.yaml

   ###############################################
   # Modify all parameters related to servoing here
   ###############################################
   use_gazebo: true # Whether the robot is started in a Gazebo simulation environment

   ## Properties of incoming commands
   command_in_type: "speed_units" # "unitless"> in the range [-1:1], as if from joystick. "speed_units"> cmds are in m/s and rad/s
   scale:
     # Scale parameters are only used if command_in_type=="unitless"
     linear:  0.6  # Max linear velocity. Meters per publish_period. Unit is [m/s]. Only used for Cartesian commands.
     rotational:  0.3 # Max angular velocity. Rads per publish_period. Unit is [rad/s]. Only used for Cartesian commands.
     # Max joint angular/linear velocity. Rads or Meters per publish period. Only used for joint commands on joint_command_in_topic.
     joint: 0.01
   # This is a fudge factor to account for any latency in the system, e.g. network latency or poor low-level
   # controller performance. It essentially increases the timestep when calculating the target pose, to move the target
   # pose farther away. [seconds]
  


1. **Launch Gazebo and UR5 Gazebo Spawner:**
   First, you need to launch Gazebo along with the UR5 Gazebo spawner. Open a terminal and run the following command:

   ```bash
   ros2 launch ur_description ur5_gazebo_launch.py
   ```

   This will start Gazebo and load the UR5 robot.

2. **Launch MoveIt! and UR5 MoveIt Spawner:**
   After launching Gazebo, open a new terminal and run the command:

   ```bash
   ros2 launch ur5_moveit spawn_ur5_launch_moveit.launch.py
   ```

   This will launch MoveIt! along with the UR5 MoveIt spawner.

3. **Visualize in Rviz:**
   After launching both Gazebo and MoveIt!, you can visualize the UR5 in Rviz. Open a new terminal and run:

   ```bash
   ros2 launch ur5_moveit demo.launch.py
   ```

   This command will open Rviz and you should be able to see the UR5 robot model. You can interact with the robot in Rviz to plan and execute motions.

4. **Add Visualization Tools:**
   As mentioned in the note, you need to add other visualization tools to Rviz. Here are the steps to add tools:
   - Click on the "Panels" tab in Rviz.
   - Select "MoveIt" from the drop-down menu.
   - A MoveIt MotionPlanning plugin panel should appear. You can use this panel to plan and execute motions for the UR5.
  


With these steps, you should be able to control the UR5 robotic arm using Rviz and MoveIt!. Make sure to follow the instructions carefully.

---

## Installation Instructions for Navigation (Nav2) and SLAM Toolbox

### Installations

**1. [Nav2](https://navigation.ros.org/index.html):**

   Install the Nav2 packages using your operating system’s package manager:

   ```bash
   sudo apt install ros-humble-navigation2
   sudo apt install ros-humble-nav2-bringup
   ```

**2. [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox):**

   Install the SLAM Toolbox that will be used to build the map and can also be used for localization:

   ```bash
   sudo apt install ros-humble-slam-toolbox
   ```

**3. [Robot-localization](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html):**

   Install the robot-localization, a collection of state estimation nodes, each of which is an implementation of a nonlinear state estimator for robots moving in 2D or 3D space:

   ```bash
   sudo apt install ros-humble-robot-localization
   ```

**4. Others:**

   Additional tools for your robotic setup:

   ```bash
   sudo apt install ros-humble-joint-state-publisher-gui
   sudo apt install ros-humble-xacro
   ```

---




## Mapping Using SLAM-Toolbox in ebot_nav2 Package

1. **Clone the Repository:**
   First, pull the latest repository of eYRC-2023_Cosmo_Logistic into your workspace. Check for the package `ebot_nav2` and verify the file structure.
   
![figure1](https://github.com/s0um0r0y/cosmo_logic/blob/main/ebot_nav2_pkg.png)

2. **Check SLAM-Toolbox Installation:**
   Ensure that SLAM-Toolbox is installed:

   ```bash
   ros2 pkg list | grep slam_toolbox
   ```

   If not installed, run the following command:

   ```bash
   sudo apt install ros-humble-slam-toolbox
   ```

3. **Set SLAM-Toolbox Parameters:**
   Set the parameters for SLAM-Toolbox in the file `mapper_params_online_async.yaml` located in the directory `/ebot_nav2/config/`.

   ```yaml
   # ROS Parameters
   odom_frame: odom
   map_frame: map
   base_frame: base_footprint
   scan_topic: /scan
   mode: mapping
   ```

4. **Load Parameters and Add SLAM-Toolbox Node:**
   Load the `mapper_params_online_async.yaml` file and add the SLAM-Toolbox node in the `ebot_bringup_launch.py` launch file located in the directory `/ebot_nav2/launch/`. Ensure that it's already added in the launch file.

   ```python
   # Loading the params
   declare_mapper_online_async_param_cmd = DeclareLaunchArgument(
       'async_param',
       default_value=os.path.join(ebot_nav2_dir, 'config', 'mapper_params_online_async.yaml'),
       description='Set mappers online async param file')

   # Adding SLAM-Toolbox with online_async_launch.py
   mapper_online_async_param_launch = IncludeLaunchDescription(
       PythonLaunchDescriptionSource(
           os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'),
       ),
       launch_arguments=[('slam_params_file', LaunchConfiguration('async_param'))],
   )
   ```

5. **Add Launch Configuration Object:**
   Add the following lines as the object of `LaunchDescription()` in the same launch file, at the end.

   ```python
   ld.add_action(declare_mapper_online_async_param_cmd)
   ld.add_action(mapper_online_async_param_launch)
   ```

   Save the file.

6. **Launch eBot in Gazebo, Teleop, and Navigation:**
   Launch the required components:

   ```bash
   ros2 launch ebot_description ebot_gazebo_launch.py
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ros2 launch ebot_nav2 ebot_bringup_launch.py
   ```

   You will see the output indicating successful mapping. Move the eBot using `teleop_twist_keyboard` in the warehouse to generate the complete map.

---
![figure1](https://github.com/s0um0r0y/cosmo_logic/blob/main/mapping_init.png)



---

## Saving the Map Generated by SLAM-Toolbox

1. **Build a Complete Map:**
   Ensure that the map generated has no empty cells in the arena or in the middle. Congratulations on creating your first map!

2. **Save the Map in RViz:**
   - Open RViz and navigate to Panels.
   - Under "Add New Panels," select `SlamToolboxPlugin` under `slam_toolbox`.

3. **Save the Map:**
   - In the `SlamToolboxPlugin` panel, enter the map name in the first two rows.
   - Click on both "Save Map" and "Serialize Map."

   This action will save the map in the currently opened directory, i.e., `/colcon_ws/`.

4. **Copy Map Files to `ebot_nav2` Package:**
   - Locate the saved map files (`map.data`, `map.pgm`, `map.posegraph`, and `map.yaml`) in the `/colcon_ws/`.
   - Copy these files to the `maps/` directory of the `ebot_nav2` package, i.e., `/ebot_nav2/maps/`.

5. **Update Map Name in Launch Files:**
   - Update the map name in the `ebot_bringup_launch.py` launch file located in `/ebot_nav2/launch/`.

   ```python
   declare_map_yaml_cmd = DeclareLaunchArgument(
       'map',
       default_value=os.path.join(ebot_nav2_dir, 'maps', 'map_name.yaml'),  ## Update the map config here
       description='Full path to map yaml file to load')
   ```

6. **Update Map Name in Parameters:**
   - Update the map name in the `nav2_params.yaml` launch file located in `/ebot_nav2/params/`.

   ```yaml
   map_server:
     ros__parameters:
       use_sim_time: True
       yaml_filename: "map_name.yaml"  ## Update the map name here
   ```

7. **Rebuild the `ebot_nav2` Package:**
   - Since you added map files to the `ebot_nav2` package, you need to rebuild it.

   ```bash
   colcon build
   ```

   OR

   ```bash
   colcon build --symlink-install
   ```

---

# Task 1A - Object Pose Estimation

## Task Objective:
The goal of this task is to perform pose estimation of package boxes using Aruco marker detection. The task involves locating package boxes in a warehouse using computer vision, specifically Aruco detection. Each package box is equipped with a unique Aruco marker. The objective is to selectively identify boxes within the robot arm's reach. Follow the instructions to set up your workspace and prepare the robotic arm for its first movement.

## Instructions:

### 1. **Prepare Your Workspace:**
   Set up your ROS workspace and ensure all necessary dependencies are installed.

### 2. **Robot Arm and Warehouse Environment:**
   You will be provided with a UR5 robotic arm model situated in a warehouse. The warehouse contains racks and package boxes. The task involves detecting the Aruco markers on these boxes.

### 3. **Aruco Marker Detection:**
   - Utilize the OpenCV library to detect Aruco markers on the package boxes.
   - Implement logic to differentiate between different markers and uniquely identify each package box.

### 4. **Pose Estimation:**
   - Once Aruco markers are detected, employ pose estimation techniques to determine the position and orientation of each package box.
   - The goal is to calculate the transformation between the Aruco marker's center position on the package box and the `base_link` of the robot arm.

### 5. **Visualization in RViz:**
   - Publish the calculated transform between Aruco marker centers and the `base_link` of the robot arm.
   - Visualize the detected package boxes and their poses in RViz for verification.

### 6. **Logical Mathematics:**
   - Apply logical and mathematical reasoning to ensure accurate pose estimation.
   - Handle different orientations, distances, and configurations of the Aruco markers.

### 7. **Documentation:**
   - Clearly document your code, explaining the logic behind marker detection and pose estimation.
   - Provide details on the mathematical concepts used for pose estimation.

### 8. **Testing and Validation:**
   - Test the pose estimation on different scenarios within the warehouse.
   - Validate the accuracy of the detected poses by comparing them with ground truth if available.

### 9. **Submission:**
   - Submit your well-documented code along with any supporting files.
   - Include instructions on how to run and test your code.

### 10. **Additional Challenge (Optional):**
   - Implement a method to dynamically update the pose estimation as the robot arm moves or as the environment changes.
   - Explore advanced computer vision techniques for more robust detection and pose estimation.


![figure1](https://github.com/s0um0r0y/cosmo_logic/blob/main/task1a_arena.png)

# Task 1A - Instructions

**Objective:**
The objective of this task is to develop a Python script for detecting Aruco markers on package boxes using RGB and Depth image data from the robot's camera. The script should publish the transform between the Aruco marker's center position and the `base_link` frame of the robot arm. Additionally, the detected Aruco tags and their center points should be displayed on the RGB image using the OpenCV library.

**Instructions:**

1. **Boilerplate Script:**
   - Start with the provided boilerplate script named `task1a.py` located in the `scripts` folder of the `ur_description` package. This script provides guidance on the steps and instructions to complete Task 1A.

2. **Dependencies Installation:**
   - Ensure that the required ROS packages are installed by running the following command in the terminal:

     ```bash
     sudo apt-get install ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller ros-humble-controller-manager
     ```

3. **Environment Setup:**
   - Launch the robot in Gazebo and open RViz by running the following commands in separate terminals:

     ```bash
     ros2 launch ur_description ur5_gazebo_launch.py
     ```

     ```bash
     ros2 launch ur_description spawn_ur5_launch.py
     ```

   - Keep these terminals running.

4. **Run the Python Script:**
   - Execute your Task 1A Python script in a new terminal. Replace the package name and filename if you are not using the provided boilerplate script.

     ```bash
     ros2 run ur_description task1a.py
     ```

5. **Visualization in RViz:**
   - In RViz:
     - Set the fixed frame as "world."
     - Add a new display or press Ctrl + N.
     - Select "TF" in the "By display type" section.
     - Add "PointCloud2" from the "By display type" section.

   - You will see all TF frames published on the "world" frame. Uncheck 'All Enabled' and only select `obj_<marker_id>` (the final box frame you are publishing) in the TF frames list.

6. **Expected Output:**
   - Compare the execution of your Python script with TF in RViz. The RViz window should display the Aruco marker's transform in relation to the robot arm's `base_link` frame.
   ![figure1](https://github.com/s0um0r0y/cosmo_logic/blob/main/img5.png)

   - The script should also display Aruco tags and their center points on the RGB image.
   ![figure1](https://github.com/s0um0r0y/cosmo_logic/blob/main/img6.png)
   ![figure1](https://github.com/s0um0r0y/cosmo_logic/blob/main/img7.png)
7. **Documentation:**
   - Ensure that your code is well-documented, explaining the logic behind Aruco marker detection and pose estimation.

8. **Testing and Validation:**
   - Test the script on different scenarios within the warehouse.
   - Validate the accuracy of the detected poses.


10. **Additional Notes:**
   - Use OpenCV2 for image processing. Other libraries can be chosen if confident but may not receive support from the e-Yantra team.
   - Handle duplicate boxes with the same Aruco IDs to avoid processing boxes that are not in the reach of the robot arm.



Best of luck with your implementation! If you have any questions or need assistance, feel free to reach out.

## TEAM MEMBERS ##
| Team ID | Name                   | Branch | Email ID                                |
|---------|------------------------|--------|-----------------------------------------|
| CL#2277 | Soumo Roy              | ECE    | soumo.roy2021@vitstudent.ac.in           |
| CL#2277 | Joel Viju              | ECE    | joelviju.v2021@vitstudent.ac.in         |
| CL#2277 | Anirudh Singareddy     | CSE    | singareddy.anirudh2021@vitstudent.ac.in |
| CL#2277 | Rohan Raj              | ECE    | rohan.raj2021@vitstudent.ac.in           |






