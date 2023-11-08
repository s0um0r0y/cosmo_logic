#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw  # Import custom service message
import math, statistics

# Define a class for your ROS2 node
class MyRobotDockingController(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('my_robot_docking_controller')

        # Create a callback group for managing callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to odometry data for robot pose information
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)

        # Subscribe to ultrasonic sensor data for distance measurements
        self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
        self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)
        # Add another one here


        # Create a ROS2 service for controlling docking behavior, can add another custom service message
        self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)

        # Create a publisher for sending velocity commands to the robot
        self.vel_pub= self.create_publisher(Twist,'cmd_vel',10)

        # Initialize all  flags and parameters here
        self.is_docking = False
        self.robot_pose = [0.0,0.0,0.0]
        self.dock_pose = [0.0,0.0]
        self.usrleft_value= 0.0
        self.usrright_value= 0.0
        self.dock_allign= False

        # Initialize a timer for the main control loop
        self.controller_timer = self.create_timer(0.1, self.controller_loop)

    # Callback function for odometry data
    def odometry_callback(self, msg):
        # Extract and update robot pose information from odometry message
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = yaw

    # Callback function for the left ultrasonic sensor
    def ultrasonic_rl_callback(self, msg):
        self.usrleft_value = msg.range

    # Callback function for the right ultrasonic sensor
    def ultrasonic_rr_callback(self, msg):
        self.usrleft_value = msg.range

    # Utility function to normalize angles within the range of -π to π (OPTIONAL)
    def normalize_angle(self, angle):
        while angle>math.pi:
            angle-=2*math.pi
        while angle<-math.pi:
            angle+=2*math.pi
        
        return angle

    # Main control loop for managing docking behavior

    def controller_loop(self):
        a_speed=0.0  #angular speed
        l_speed=0.0  #linear speed
        # The controller loop manages the robot's linear and angular motion 
        # control to achieve docking alignment and execution
        if self.is_docking:
            # ...
            # Implement control logic here for linear and angular motion
            # For example P-controller is enough, what is P-controller go check it out !
            # ...
            tar_angle=self.dock_pose[1]  #target angle
            ang_err=self.normalize_angle(tar_angle-self.robot_pose[2]) #angular error
            a_speed=0.7*ang_err
            vel_msg=Twist()
            vel_msg.angular.z=a_speed
            self.vel_pub.publish(vel_msg)

            if abs(ang_err) < 0.03:
                self.dock_allign=True
                self.get_logger().info("Robot is aligned for docking.")
            else:
                self.get_logger().info("Aligning the robot according to the rack")

            if self.dock_allign==True and self.l_dock==True:  
                l_speed=self.calculate_linear_correction()
                if l_speed==0.0:
                    vel_msg=Twist()
                    vel_msg.linear.x=0.0
                    self.vel_pub.publish(vel_msg)
                    self.is_docking=True
                    self.get_logger().info("Docking complete.")

                else:
                    vel_msg=Twist()
                    vel_msg.linear.x=l_speed
                    self.vel_pub.publish(vel_msg)
                    self.get_logger().info(f"Linear speed correction:{l_speed}")

    def calculate_linear_correction(self):
        rr_distance = min(self.usrleft_value, self.usrright_value) #rear distance
        if rr_distance>1.0:
            self.l_speed=0.2
        if rr_distance<0.15:
            self.l_speed=0.0
        else:
            self.l_speed=-0.2
        return self.l_speed
    

    # Callback function for the DockControl service
    def dock_control_callback(self, request, response):
        # Extract desired docking parameters from the service request
        self.is_docking=True

        # Reset flags and start the docking process
        self.dock_allign=False

        ####### add client server code here ############

        self.is_docking=True
        self.l_dock=request.linear_dock
        self.o_dock=request.orientation_dock
        self.dock_pose=[request.distance,request.orientation]

        # Log a message indicating that docking has started
        self.get_logger().info("Docking started!")

        # Create a rate object to control the loop frequency
        rate = self.create_rate(2, self.get_clock())

        # Wait until the robot is aligned for docking
        while not self.dock_aligned:
            self.get_logger().info("Waiting for alignment...")
            rate.sleep()

        # Set the service response indicating success
        response.success = True
        response.message = "Docking control initiated"
        return response

# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    rclpy.init(args=args)

    my_robot_docking_controller = MyRobotDockingController()

    executor = MultiThreadedExecutor()
    executor.add_node(my_robot_docking_controller)

    executor.spin()

    my_robot_docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
