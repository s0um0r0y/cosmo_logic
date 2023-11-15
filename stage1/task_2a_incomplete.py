import rospy
import moveit_commander
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped

rospy.init_node('moveit_ur5_control', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"  # Update this with your specific group name
move_group = moveit_commander.MoveGroupCommander(group_name)

def publish_transform(box_name, translation, rotation):
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    transform = geometry_msgs.msg.TransformStamped()

    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "base_link"  # Assuming the base frame of UR5
    transform.child_frame_id = box_name
    transform.transform.translation.x = translation[0]
    transform.transform.translation.y = translation[1]
    transform.transform.translation.z = translation[2]
    transform.transform.rotation.x = rotation[0]
    transform.transform.rotation.y = rotation[1]
    transform.transform.rotation.z = rotation[2]
    transform.transform.rotation.w = rotation[3]

    broadcaster.sendTransform(transform)

def move_to_box(box_name):
    move_group.set_pose_reference_frame("base_link")
    move_group.set_goal_tolerance(0.01)
    pose_target = PoseStamped()
    pose_target.header.frame_id = "base_link"  # Assuming the base frame of UR5
    pose_target.header.stamp = rospy.Time.now()
    pose_target.pose.position.x = 0.0  # Update with box's position
    pose_target.pose.position.y = 0.0  # Update with box's position
    pose_target.pose.position.z = 0.0  # Update with box's position
    pose_target.pose.orientation.x = 0.0
    pose_target.pose.orientation.y = 0.0
    pose_target.pose.orientation.z = 0.0
    pose_target.pose.orientation.w = 1.0
    move_group.set_pose_target(pose_target)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
box1_translation = [0.5, 0.0, 0.3]
box1_rotation = [0.0, 0.0, 0.0, 1.0]

box2_translation = [0.0, 0.5, 0.3]
box2_rotation = [0.0, 0.0, 0.0, 1.0]

box3_translation = [0.5, 0.5, 0.3]
box3_rotation = [0.0, 0.0, 0.0, 1.0]

publish_transform("box1", box1_translation, box1_rotation)
publish_transform("box2", box2_translation, box2_rotation)
publish_transform("box3", box3_translation, box3_rotation)

move_to_box("box1")
move_to_box("box2")
move_to_box("box3")
