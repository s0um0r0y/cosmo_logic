import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def publish_box_transforms():
    rospy.init_node('box_transforms_publisher')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    box1_transform = TransformStamped()
    box1_transform.header.frame_id = "world"
    box1_transform.child_frame_id = "box1"
    box1_transform.transform.translation.x = 0.5  # Modify the x, y, z values accordingly
    box1_transform.transform.translation.y = 0.0
    box1_transform.transform.translation.z = 0.3
    box1_transform.transform.rotation.w = 1.0  # No rotation (unit quaternion)

    box2_transform = TransformStamped()
    # Define transform for box2 similarly

    box3_transform = TransformStamped()
    # Define transform for box3 similarly

    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
    tf_broadcaster.sendTransform([box1_transform, box2_transform, box3_transform])

    rospy.spin()

if __name__ == '__main__':
    try:
        publish_box_transforms()
    except rospy.ROSInterruptException:
        pass
