from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_pose_goal")

    # Declare parameters for position and orientation
    node.declare_parameter("position", [0.24895,-0.45569, 0.65625])
    node.declare_parameter("quat_xyzw", [0.071078, 0.025473, 0.020497, 0.070625])
    node.declare_parameter("position1", [-0.37, 0.12, 0.397])
    node.declare_parameter("quat_xyzw1", [0.5,0.5,0.5,0.5])
    node.declare_parameter("position2", [0.194, -0.43, 0.701])
    node.declare_parameter("quat_xyzw2", [0.5,0.5,0.5,0.5])
    node.declare_parameter("cartesian", False)

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s)
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
        f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
    )
    moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2.wait_until_executed()
    node.get_logger().info(
         f"Moving to {{position: {list(position1)}, quat_xyzw: {list(quat_xyzw1)}}}"
    )
    gripper_control = self.create_client(AttachLink, '/GripperMagnetON')
    
    while not gripper_control.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('EEF service not available, waiting again...')

    req = AttachLink.Request()
    req.model1_name =  <Specify the box name>      
    req.link1_name  = 'link'       
    req.model2_name =  'ur5'       
    req.link2_name  = 'wrist_3_link'  

    
    moveit2.move_to_pose(position=position1, quat_xyzw=quat_xyzw1, cartesian=cartesian)
    moveit2.wait_until_executed()
    node.get_logger().info(
        f"Moving to {{position: {list(position2)}, quat_xyzw: {list(quat_xyzw2)}}}"
    )
    gripper_control = self.create_client(DetachLink, '/GripperMagnetOFF')
    
    while not gripper_control.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('EEF service not available, waiting again...')

    req = AttachLink.Request()
    req.model1_name =  <Specify the box name>      
    req.link1_name  = 'link'       
    req.model2_name =  'ur5'       
    req.link2_name  = 'wrist_3_link'  
    
    moveit2.move_to_pose(position=position2, quat_xyzw=quat_xyzw2, cartesian=cartesian)
    moveit2.wait_until_executed()
    node.get_logger().info(
        f"Moving to {{position: {list(position1)}, quat_xyzw: {list(quat_xyzw1)}}}"
    )
    moveit2.move_to_pose(position=position1, quat_xyzw=quat_xyzw1, cartesian=cartesian)
    moveit2.wait_until_executed()
   
    gripper_control.call_async(req)


    node.get_logger().info(
        f"Completed"
    )
    rclpy.shutdown()
    exit(0)


if _name_ == "_main_":
    main()
