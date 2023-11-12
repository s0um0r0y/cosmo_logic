#! /usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from ebot_docking_boilerplate import MyRobotDockingController
from ebot_docking.srv import DockSw  
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
import time
from ebot_docking.srv import DockSw  # Import custom service message
from linkattacher_msgs.srv import AttachLink, DetachLink 
import math, statistics
import threading

class navigation(Node):

    def __init__(self):
        super().__init__('navigator_node')
        self.callback_group = ReentrantCallbackGroup()
        self.attach_cli = self.create_client(AttachLink, '/ATTACH_LINK')
        while not self.attach_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Link attacher service not available, waiting again...')
        self.req = AttachLink.Request()
        self.req.model1_name =  'ebot'     
        self.req.link1_name  = 'ebot_base_link'       
        self.req.model2_name =  'rack1'       
        self.req.link2_name  = 'link'   
        self.detach_cli= self.create_client(DetachLink, '/DETACH_LINK')
        while not self.detach_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting...')
        self.reqd=DetachLink.Request()
        self.reqd.model1_name='ebot'         
        self.reqd.link1_name='ebot_base_link'
        self.reqd.model2_name= 'rack1'
        self.reqd.link2_name= 'link' 
        self.client = self.create_client(DockSw, 'dock_control')  
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting...')
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.navigator = BasicNavigator()
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = 0.0
        self.initial_pose.pose.position.y = 0.0
        self.initial_pose.pose.orientation.w = 0.0
        self.navigator.setInitialPose(self.initial_pose)
        self.navigator.waitUntilNav2Active()
        self.robot_pose=[0.0,0.0,0.0]
        self.linear_correction = True
        self.angular_correction = True
        self.distance = 1.22 
        self.goal_orientation = 3.14  
        self.rack_number = "Rack1"
        self.distance2=0.7
        self.rack_number = "Rack2"
        self.goal_pose2 = PoseStamped() # [2.0, -7.0, -1.57]
        self.goal_pose2.header.frame_id = 'map'
        self.goal_pose2.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose2.pose.position.x = -1.0
        self.goal_pose2.pose.position.y = -2.555
        self.goal_pose2.pose.position.z = 0.0
        self.goal_pose2.pose.orientation.z = -0.7071
        self.goal_pose2.pose.orientation.w = 0.7071  
        self.goal_pose3 = PoseStamped()
        self.goal_pose3.header.frame_id = 'map'
        self.goal_pose3.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose3.pose.position.x = 0.0
        self.goal_pose3.pose.position.y = 0.0
        self.goal_pose3.pose.position.z = 0.0
        self.goal_pose3.pose.orientation.z = -0.7071
        self.goal_pose3.pose.orientation.w = 0.7071  

    def send_docking_request(self,linear_correction, angular_correction, distance, goal_orientation, rack_number):
        self.request = DockSw.Request()
        self.request.linear_dock = linear_correction
        self.request.orientation_dock = angular_correction
        self.request.distance = distance
        self.request.orientation = goal_orientation
        self.request.rack_no = rack_number
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        self.response = self.future.result()
        if self.future.result() is not None:
                self.response = self.future.result()
                self.get_logger().info('Docking request was successful: %s' % (self.response.success))
                self.get_logger().info('Message: %s' % (self.response.message))
                self.success = True
        else:
                self.get_logger().error('Service call failed.')
                self.success= False
    

    def odometry_callback(self,msg):
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        self.quaternion_array = msg.pose.pose.orientation
        orientation_list = [self.quaternion_array.x, self.quaternion_array.y, self.quaternion_array.z, self.quaternion_array.w]
        self.roll,self.pitch,self.yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = self.yaw
        

    def feedback(self):
        
        i=0
        
        while not self.navigator.isTaskComplete():
            
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
    def nav_Calling(self):
         self.goal_pose1 = PoseStamped() 
         self.goal_pose1.header.frame_id = 'map'
         self.goal_pose1.header.stamp = self.navigator.get_clock().now().to_msg()
         self.goal_pose1.pose.position.x = -1.02
         self.goal_pose1.pose.position.y = 4.39
         self.goal_pose1.pose.position.z = 0.0    
         self.goal_pose1.pose.orientation.z = 0.7071
         self.goal_pose1.pose.orientation.w = 0.7071
         self.navigator.goToPose(self.goal_pose1)
         self.feedback()
         self.send_docking_request(self.linear_correction,self.angular_correction,self.distance,self.goal_orientation, self.rack_number)
         if self.success==True:
              print('checking')
              while not(self.robot_pose[0]>1.02 and self.robot_pose[0]<1.26):
                   print(self.robot_pose)
                   rclpy.spin_once(self)
              b=-1000
              while(b<1000):
                   b=b+1
              print('completed')
              self.future2 = self.attach_cli.call_async(self.req)
              rclpy.spin_until_future_complete(self, self.future2)
              print(self.future2.result)
              print(self.future)
              print('attachment completed')
         self.navigator.goToPose(self.goal_pose2)
         self.feedback()
         print('successfully travelled to point 2')
         self.send_docking_request(self.linear_correction,self.angular_correction,self.distance2,self.goal_orientation,self.rack_number)
         print('checking')
         while not(self.robot_pose[0]<0.5 and self.robot_pose[0]>0.4):
                   rclpy.spin_once(self)
                   print(self.robot_pose)
         print('last docking completed')
         self.future4 = self.detach_cli.call_async(self.reqd)
         rclpy.spin_until_future_complete(self, self.future4)
         print('detachment completed')
         self.navigator.goToPose(self.goal_pose3)
         self.feedback()


         self.navigator.lifecycleShutdown()
def main(args=None):
    rclpy.init(args=args)

    navigator_node = navigation()
    
    executor = MultiThreadedExecutor()
    
    executor.add_node(navigator_node)
    navigator_node.nav_Calling()
    executor.spin()
    navigator_node.destroy_node()
    rclpy.shutdown()
    executor.shutdown()
if __name__ == '__main__':
    main()
         
