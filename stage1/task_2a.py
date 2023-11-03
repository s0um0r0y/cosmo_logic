import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_pose_goal")

    # Declare parameters for position and orientation
    node.declare_parameter("position1", [0.4,0.76,0.8])
    node.declare_parameter("position", [0.35, 0.1, 0.18])
    node.declare_parameter("quat_xyzw1", [0.4907, 0.4832, 2.6198, 0.0])
    node.declare_parameter("quat_xyzw", [0.5, 0.5, 0.5, 0.5])
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
    position = node.get_parameter("position1").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw1").get_parameter_value().double_array_value
    position1= node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw1 = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value

    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value

    # Move to pose
    node.get_logger().info(
        f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
        f"Moving to {{position1: {list(position)}, quat_xyzw1: {list(quat_xyzw)}}}"
    )
    moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2.move_to_pose(position=position1, quat_xyzw=quat_xyzw1, cartesian=cartesian)
    moveit2.wait_until_executed()

    rclpy.shutdown()
    exit(0)


if _name_ == "_main_":
    main()
