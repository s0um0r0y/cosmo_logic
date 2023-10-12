import os
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import geomtery_msgs.msg
import moveit_commander
import numpy as np
from cv_bridge import CvBridge
from tf.transformations import quaternion_from_euler
import time
# from calibirate_position import calibirator
import json

class Object_detector:


    def callback(self,data):
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(data,"bgr8")
        cv2.namedWindow('Live Video')
        cv2.imshow('Live Video',image)
        cv2.waitKey(1)

class GripperMover:
    def __init__(self,
                 gripper_commander_group_name: str):
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_commander_group_name)
    
    def commander(self,command : str):
        self.gripper_group.set_named_target(command)
        self.gripper_group.plan()
        self.gripper_group.go(wait= True)
    
    def open(self):
        self.commander('open')
    
    def close(self):
        self.commander('close')




class MoveRobot:
    def __init__(self,robot_commander_group_name: str = 'robot1',
                 gripper_commander_group_name: str = 'robot1_gripper'):
        
        self.robot_group = moveit_commander.MoveGroupCommander(robot_commander_group_name)
        self.robot_group.set_max_acceleration_scaling_factor(1.0)
        self.robot_group.set_max_velocity_scaling_factor(1.0)
        self.robot_group.set_goal_joint_tolerance(0.01)
        # self.robot_group.set_planner_id('ABITstar')
        self.robot_group.set_num_planning_attempts(999)
        self.robot_group.set_planning_time(20)
        self.robot_group.set_goal_tolerance(0.001)
        self.gripper_group = GripperMover(gripper_commander_group_name = gripper_commander_group_name)
    
    def take_home(self):
        joint_target = [4.368849670658683,
                        -1.6745898414578253,
                        2.0752037770433738,
                        4.300547921616338,
                        4.709010387553134,
                        -5.148826192873531]
        
        self.robot_group.go(joint_target,wait = True)
        
        
    def take_pick_position(self):
        joint_target = [1.5708,
                        -1.6745898414578253,
                        2.0752037770433738,
                        4.300547921616338,
                        4.709010387553134,
                        -5.148826192873531]
        self.robot_group.go(joint_target,wait = True)
    
    def position(self,
                 angle: float,
                 x : float,
                 y : float,
                 z : float):
        pose_target = geometry_msgs.msg.Pose()
        
        # setting custom orientation for the robot
        q = quaternion_from_euler(math.radians(-180),
                                  math.radians(0),
                                  math.radians(angle))
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z
        pose_target.orientation.x = q[0]
        pose_target.orientation.y = q[1]
        pose_target.orientation.z = q[2]
        pose_target.orientation.w = q[3]
        
        # self.robot_group.set_position_target([pose_target.position.x,
        #                                       pose_target.position.y,
        #                                       pose_target.position.z])
        self.robot_group.set_pose_target(pose_target)
        self.robot_group.plan()
        self.robot_group.go(wait=True)
        
        # self.robot_group.set_orientation_target([pose_target.orientation.x,
        #                                          pose_target.orientation,y,
        #                                          pose_target.orientation.z,
        #                                          pose_target.orientation.w])
        # self.robot_group.plan()
        # self.robot_group.go()
    
    def pick_at_position(self,angle , x,y):
        if angle >= 90:
            angle = angle -90
        print('------------- {text} ----------------'.format(text = "TAKING PICK POSITION"))
        self.take_pick_position()
        print('------------- {text} ----------------'.format(text = "OPENING THE GRIPPER"))
        self.gripper_group.commander('open')
        # time.sleep()
        print('------------- {text} ----------------'.format(text = "GETTING TO THE POSITION OF OBJECT"))
        self.position(angle=-90,
                      x = x,
                      y= y,
                      z= 1.2)
        print('------------- {text} ----------------'.format(text = "FIXING ANGLE"))
        self.position(angle=-90 - angle,
                      x = x,
                      y= y,
                      z = 1.2)
        print('------------- {text} ----------------'.format(text = "GETTING CLOSE TO OBJECT"))
        self.position(angle=-90 - angle,
                      x = x,
                      y= y,
                      z = 0.88)
        print('------------- {text} ----------------'.format(text = "GRABBING THE OBJECT"))
        self.gripper_group.commander('close')
        time.sleep(3)
        print('------------- {text} ----------------'.format(text = "GRABBING SUCCESSFUL"))
        self.position(angle=-90 - angle,
                      x = x,
                      y= y,
                      z = 1.2)
        
    def place_at_home(self):
        print('------------- {text} ----------------'.format(text = "MOVING TO HOME"))
        self.take_home()
        print('------------- {text} ----------------'.format(text = "RELEASING OBJECT"))
        self.gripper_group.commander('open')
        time.sleep(3)
        print('------------- {text} ----------------'.format(text = "CLOSING THE GRIPPER"))
        self.gripper_group.commander('close')
        
        
        
        
        

if __name__== '__main__':
    rospy.init_node("move_group_python_interface",anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    print(robot.get_group_names())
    robot1 = MoveRobot()
    robot1.take_home()
    
    defined_positions=  [
                 [-0.1,0.7,1.05],
                 [-0.4,0.9,1.05],
                 [0.4,0.9,1.05],
                 [0.0,0.6,1.05],
                 [-0.4,0.6,1.05],
                 [0.4,0.6,1.05],
                 [-0.4,0.15,1.05]]
    for i in defined_positions:
        print('POSITION : changing')
        rospy.sleep(5)
        robot1.position(i[0],
                        i[1],
                        i[2])
        print('POSITION CHANGED : ',i)
