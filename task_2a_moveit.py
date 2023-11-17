from std_srvs.srv import Trigger
from threading import Thread

from math import cos, sin
import math, time
from copy import deepcopy
import rclpy
import tf2_ros
from tf2_ros import TransformListener,TFMessage
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped,TransformStamped
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.duration import Duration

class Posecontroller(Node):
    def __init__(self):
        super().__init__('pose_controller')
        self.twist_pub=self.create_publisher(TwistStamped,'/pose_node/delta_twist_cmds',10)
        self.tf_buffer=tf2_ros.Buffer(Duration(seconds=1))
        self.tf_listener=tf2_ros.TransformListener(self.tf_buffer,self)

        # self.target_pose=[translation.x,translation.y,translation.z]
        # self.target_pose=[0.3014,0.1023,0.629]
        self.stop_threshold=0.2
        self.subscription1=None

        self.timer=self.create_timer(0.02,self.pose_reach_target,ReentrantCallbackGroup())

    def transform_callback(self,msg):
        try:
            # print(msg)
            for transform_stamped in msg.transforms:
                self.translation = transform_stamped.transform.translation
                self.rotation = transform_stamped.transform.rotation
                self.frame_id = transform_stamped.header.frame_id
                # self.child_frame_id = transform_stamped.header.child_frame_id

                # self.get_logger().info(
                #     f"Received transform: Translation({self.translation.x}, {self.translation.y}, {self.translation.z}), "
                #     f"Rotation({self.rotation.x}, {self.rotation.y}, {self.rotation.z}, {self.rotation.w}) "
                #     f"from frame '{self.frame_id}' to frame '{self.child_frame_id}'"
                # )

                self.target_pose=[self.translation.x,self.translation.y,self.translation.z]
        except Exception as e:
                self.get_logger().error(f"Error in transform_callback: {e}")

    def transform_subscription(self):
        self.subscription1=self.create_subscription(TFMessage,'/tf',self.transform_callback,10)

    def pose_reach_target(self)
         
              transform=self.tf_buffer.lookup_transform('base_link','obj_1',rclpy.time.Time().to_msg())
              translation=transform.transform.translation

              current_pose=[translation.x,translation.y,translation.z]
              print(current_pose)
              node.declare_parameter("position", self.target_pose[0]-current_pose[0])
              node.declare_parameter("quat_xyzw", [0.5, 0.5, 0.5, 0.5])
              node.declare_parameter("position1", self.target_pose[1]-current_pose[1])
              node.declare_parameter("quat_xyzw1", [0.5,0.5,0.5,0.5])
              node.declare_parameter("position2", self.target_pose[2]-current_pose[2])
              node.declare_parameter("quat_xyzw2", [0.5,0.5,0.5,0.5])
              node.declare_parameter("cartesian", False)

              callback_group = ReentrantCallbackGroup()

              moveit2 = MoveIt2(
                node=node,
                joint_names=ur5.joint_names(),
                base_link_name=ur5.base_link_name(),
                end_effector_name=ur5.end_effector_name(),
                group_name=ur5.MOVE_GROUP_ARM,
                callback_group=callback_group,
              )
              
            
             executor = rclpy.executors.MultiThreadedExecutor(2)
             executor.add_node(node)
             executor_thread = Thread(target=executor.spin, daemon=True, args=())
             executor_thread.start()

    # Get parameters
            position= node.get_parameter("position").get_parameter_value().double_array_value
            quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
            position1= node.get_parameter("position1").get_parameter_value().double_array_value
            quat_xyzw1= node.get_parameter("quat_xyzw1").get_parameter_value().double_array_value
            position2= node.get_parameter("position2").get_parameter_value().double_array_value
            quat_xyzw2= node.get_parameter("quat_xyzw2").get_parameter_value().double_array_value
            cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value

    # Move to pose
    
         node.get_logger().info(
        f"  Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
        )
         moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
         moveit2.wait_until_executed()
         node.get_logger().info(
            f"Moving to {{position: {list(position1)}, quat_xyzw: {list(quat_xyzw1)}}}"
        )
        moveit2.move_to_pose(position=position1, quat_xyzw=quat_xyzw1, cartesian=cartesian)
        moveit2.wait_until_executed()
        node.get_logger().info(
           f"Moving to {{position: {list(position2)}, quat_xyzw: {list(quat_xyzw2)}}}"
        )
        moveit2.move_to_pose(position=position2, quat_xyzw=quat_xyzw2, cartesian=cartesian)
        moveit2.wait_until_executed()
        node.get_logger().info(
          f"Moving to {{position: {list(position1)}, quat_xyzw: {list(quat_xyzw1)}}}"
        )
        moveit2.move_to_pose(position=position1, quat_xyzw=quat_xyzw1, cartesian=cartesian)
        moveit2.wait_until_executed()

        node.get_logger().info(
          f"Completed"
        )
    def main():
    rclpy.init()
    node = Posecontroller()
    node.transform_subscription()
    node.pose_reach_target()
    rclpy.spin(node=node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()



