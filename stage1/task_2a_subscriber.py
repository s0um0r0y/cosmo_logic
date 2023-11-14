import rospy
import moveit_commander
from geometry_msgs.msg import Pose

def move_to_box_position(box_name):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_ur5_to_box')
    robot = moveit_commander.RobotCommander()
    group_name = "manipulator"  # Adjust according to your robot's MoveIt! setup
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Get the pose of the box from TF
    tf_listener = tf2_ros.TransformListener()
    try:
        trans = tf_listener.lookup_transform("base_link", box_name, rospy.Time(0), rospy.Duration(1.0))
        box_pose = Pose()
        box_pose.position.x = trans.transform.translation.x
        box_pose.position.y = trans.transform.translation.y
        box_pose.position.z = trans.transform.translation.z
        box_pose.orientation.x = trans.transform.rotation.x
        box_pose.orientation.y = trans.transform.rotation.y
        box_pose.orientation.z = trans.transform.rotation.z
        box_pose.orientation.w = trans.transform.rotation.w

        # Plan and execute the robot movement to the box position
        move_group.set_pose_target(box_pose)
        move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr("TF Exception: %s", e)

    rospy.sleep(1)
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        # Move to box1 position
        move_to_box_position("box1")
        # Repeat for box2 and box3
        move_to_box_position("box2")
        move_to_box_position("box3")
    except rospy.ROSInterruptException:
        pass
