#!/usr/bin/env python3
"""
Example of using MoveIt 2 Servo to perform a circular motion.
`ros2 run pymoveit2 ex_servo.py`
"""

from std_srvs.srv import Trigger
from math import cos, sin
import math, time
from copy import deepcopy
import rclpy
import tf2_ros
from tf2_ros import TransformListener
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
from rclpy.duration import Duration

class Servocontroller(Node):
    def __init__(self):
        super().__init__('servo_controller')
        self.twist_pub=self.create_publisher(TwistStamped,'/servo_node/delta_twist_cmds',10)
        self.tf_buffer=tf2_ros.Buffer(Duration(seconds=1))
        self.tf_listener=tf2_ros.TransformListener(self.tf_buffer,self)

        # self.target_pose=[translation.x,translation.y,translation.z]
        # self.target_pose=[0.3014,0.1023,0.629]
        self.stop_threshold=0.2

        self.timer=self.create_timer(0.02,self.servo_reach_target,ReentrantCallbackGroup())

    def transform_callback(self,msg):
        self.translation=msg.transform.translation
        self.rotation=msg.transform.rotation
        self.frame_id=msg.header.frame_id
        self.child_frame_id=msg.child_frame_id

        self.get_logger().info(
            f"Received transform: Translation({self.translation.x}, {self.translation.y}, {self.translation.z}), "
            f"Rotation({self.rotation.x}, {self.rotation.y}, {self.rotation.z}, {self.rotation.w}) "
            f"from frame '{self.frame_id}' to frame '{self.child_frame_id}'"
        )

        self.target_pose=[self.translation.x,self.translation.y,self.translation.z]

    def transform_subscription(self):
        self.subscriptions=self.create_subscription(TransformStamped,'tf',self.transform_callback,10)
        self.subscriptions

    def servo_reach_target(self):
            transform=self.tf_buffer.lookup_transform('base_link','tool0',rclpy.time.Time().to_msg())
            translation=transform.transform.translation

            current_pose=[translation.x,translation.y,translation.z]
            print(current_pose)

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

    def start_servo_service(self):
        start_servo_client=self.create_client(Trigger,'/servo_node/start_servo')
        if not start_servo_client.wait_for_service(timeout_sec=2):
            self.get_logger().warn("service is not available")
        else:
            request=Trigger.Request()
            future=start_servo_client.call_async(request=request)
            rclpy.spin_until_future_complete(self,future=future)

            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info("servo controller started")
                else:
                    self.get_logger().error("failed to start servo contoller: %s",future.result().message)
            else:
                self.get_logger().error("Service call failed")

def main():
    rclpy.init()
    node = Servocontroller()
    node.start_servo_service()
    node.transform_subscription()
    node.servo_reach_target()
    rclpy.spin(node=node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
