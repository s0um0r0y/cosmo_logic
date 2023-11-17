#!/usr/bin/env python3
import math
from rclpy.time import Time

from std_srvs.srv import Trigger
import rclpy
import tf2_ros
from tf2_ros import TransformListener,TFMessage
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped,TransformStamped
from pymoveit2.robots import ur5
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from pymoveit2 import MoveIt2
from threading import Thread
from rclpy.duration import Duration
from linkattacher_msgs.srv import AttachLink,DetachLink
from tf2_ros import TransformException

class Servocontroller(Node):
    def __init__(self):
        super().__init__('servo_controller')

        self.twist_pub=self.create_publisher(TwistStamped,'/servo_node/delta_twist_cmds',10)
        #self.subscription1=self.create_subscription(TFMessage,'/tf',self.transform_callback,10)
        self.tf_buffer=tf2_ros.Buffer(Duration(seconds=1))
        self.tf_listener=tf2_ros.TransformListener(self.tf_buffer,self)
        self.gripper_control = self.create_client(AttachLink, '/GripperMagnetON')
        while not self.gripper_control.wait_for_service(timeout_sec=1.0):    
            self.get_logger().info('EEF service not available, waiting again...')
        self.gripper_control_2 = self.create_client(DetachLink, '/GripperMagnetOFF')
        while not self.gripper_control_2.wait_for_service(timeout_sec=1.0):    
            self.get_logger().info('EEF service not available, waiting again...')
        self.stop_threshold=0.2
        self.target_pose=[0,0,0]
        #self.timer=self.create_timer(0.02,self.servo_reach_target,ReentrantCallbackGroup())
        self.timer = self.create_timer(1.0, self.lookup_transforms)
        self.first_transform_received = False  # Flag to track the first transform
      
    def lookup_transforms(self):
        try:
            # transform1 = self.tf_buffer.lookup_transform("base_link", "obj_1", Time().to_msg())
            transform3 = self.tf_buffer.lookup_transform("base_link", "obj_3", Time().to_msg())
            # transform49 = self.tf_buffer.lookup_transform("base_link", "obj_49", Time().to_msg())

            # Extract and store translations
            # self.translations= (transform1.transform.translation.x, transform1.transform.translation.y, transform1.transform.translation.z)
            self.translations = (transform3.transform.translation.x, transform3.transform.translation.y, transform3.transform.translation.z)
            # self.translations = (transform49.transform.translation.x, transform49.transform.translation.y, transform49.transform.translation.z)

            # Extract and store rotations
            # self.rotations = (transform1.transform.rotation.x, transform1.transform.rotation.y, transform1.transform.rotation.z, transform1.transform.rotation.w)
            self.rotations = (transform3.transform.rotation.x, transform3.transform.rotation.y, transform3.transform.rotation.z, transform3.transform.rotation.w)
            # self.rotations = (transform49.transform.rotation.x, transform49.transform.rotation.y, transform49.transform.rotation.z, transform49.transform.rotation.w)

            # Print or process the transforms as needed
            print(self.translations)
            self.first_transform_received=True
            # Stop the timer after receiving the first transform
            if self.first_transform_received==True:

                self.timer.cancel()
                self.pose_reach_target()          

        except TransformException as e:
            self.get_logger().warning(f"Failed to lookup transforms: {e}")

    '''def transform_callback(self,msg):
        try:
                print(msg)
                self.translation = msg.transforms.transform.translation
                self.rotation = msg.transforms.rotation
                self.frame_id = msg.header.frame_id

                # self.get_logger().info(
                #     f"Received transform: Translation({self.translation.x}, {self.translation.y}, {self.translation.z}), "
                #     f"Rotation({self.rotation.x}, {self.rotation.y}, {self.rotation.z}, {self.rotation.w}) "
                #     f"from frame '{self.frame_id}' to frame '{self.child_frame_id}'"
                # )

                self.target_pose=[self.translation.x,self.translation.y,self.translation.z]
                print('target pose is',self.target_pose)
        
        except Exception as e:
                self.get_logger().error(f"Error in transform_callback: {e}")'''
    
    def send_request_gripper(self,model1,link1,model2,link2):
            self.req = AttachLink.Request()
            self.req.model1_name=model1
            self.req.link1_name=link1
            self.req.model2_name=model2
            self.req.link2_name=link2
            self.future = self.gripper_control.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)
            self.response = self.future.result()
            if self.future.result() is not None:
                self.response = self.future.result()
                self.get_logger().info('Gripper request was successful: %s' % (self.response.success))
                self.get_logger().info('Message: %s' % (self.response.message))
            else:
                self.get_logger().error('Service call failed.')
    
    def send_request_degrip(self,model1,link1,model2,link2):
        self.req2= DetachLink.Request()
        self.req2.model1_name=model1
        self.req2.link1_name=link1
        self.req2.model2_name=model2
        self.req2.link2_name=link2
        self.future2 = self.gripper_control_2.call_async(self.req2)
        rclpy.spin_until_future_complete(self, self.future2)
        self.response = self.future2.result()
        if self.future2.result() is not None:
                self.response = self.future2.result()
                self.get_logger().info('Gripper request was successful: %s' % (self.response.success))
                self.get_logger().info('Message: %s' % (self.response.message))
        else:
                self.get_logger().error('Service call failed.')

    
    def servo_reach_target(self):
        try:
            transform=self.tf_buffer.lookup_transform('base_link','obj_3',rclpy.time.Time().to_msg())
            # transform=self.tf_buffer.lookup_transform('base_link','obj_1',rclpy.time.Time().to_msg())
            # transform=self.tf_buffer.lookup_transform('base_link','obj_49',rclpy.time.Time().to_msg())
            translation=transform.transform.translation

            current_pose=[translation.x,translation.y,translation.z]

            distance=math.sqrt((self.target_pose[0]-current_pose[0])**2+
                               (self.target_pose[1]-current_pose[1])**2+
                               (self.target_pose[2]-current_pose[2])**2)
            
            gain=3.0

            if distance> self.stop_threshold:
                linear_velocity=[gain*(self.target_pose[0]-current_pose[0]),
                                 gain*(self.target_pose[1]-current_pose[1]),
                                 gain*(self.target_pose[2]-current_pose[2])
                ]

                twist_msg=TwistStamped()
                twist_msg.header.frame_id='base_link'
                twist_msg.twist.linear.x=linear_velocity[0]
                twist_msg.twist.linear.y=linear_velocity[1]
                twist_msg.twist.linear.z=linear_velocity[2]
                twist_msg.header.stamp=self.get_clock().now().to_msg()
                self.twist_pub.publish(twist_msg)
        
     
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    def start_servo_service(self):
        start_servo_client=self.create_client(Trigger,'/servo_node/start_servo')
        if not start_servo_client.wait_for_service(timeout_sec=2):
            self.get_logger().warn("service is not available")
        else:
            request=Trigger.Request()
            future=start_servo_client.call_async(request)
            rclpy.spin_until_future_complete(self,future=future)

            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info("servo controller started")
                else:
                    self.get_logger().error("failed to start servo contoller: %s",future.result().message)
            else:
                self.get_logger().error("Service call failed")

    def pose_reach_target(self):
            
              self.declare_parameter("position", self.translations)
              self.declare_parameter("quat_xyzw", self.rotations)
              self.declare_parameter("cartesian", False)

              callback_group1 = ReentrantCallbackGroup()

              moveit2 = MoveIt2(
                self,
                joint_names=ur5.joint_names(),
                base_link_name=ur5.base_link_name(),
                end_effector_name=ur5.end_effector_name(),
                group_name=ur5.MOVE_GROUP_ARM,
                callback_group=callback_group1,
              )
              position= self.get_parameter("position").get_parameter_value().double_array_value
              quat_xyzw = self.get_parameter("quat_xyzw").get_parameter_value().double_array_value
              cartesian = self.get_parameter("cartesian").get_parameter_value().bool_value
    
              self.get_logger().info(
              f"  Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
              )
              moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
              moveit2.wait_until_executed()
              self.send_request_gripper('box1','link','ur5','writst_3_link')
              moveit2.move_to_pose(-0.37,0.12,0.397)
              self.get_logger().info(
             "moving to drop pose"
              )
              moveit2.wait_until_executed()
       

def main():
        rclpy.init()
        node = Servocontroller()

        #node.start_servo_service()
        #node.servo_reach_target()
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(node)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        rclpy.spin(node)
        rclpy.shutdown()
        
if __name__ == "__main__":
        main()


    
