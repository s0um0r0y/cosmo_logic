#! /usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration



def main():
    rclpy.init()

    navigator = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    
     
    goal_pose1 = PoseStamped() #1.8, 1.5, 1.57
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 1.8
    goal_pose1.pose.position.y = 1.5
    goal_pose1.pose.position.z = 0.0    
    goal_pose1.pose.orientation.z = 0.7071
    goal_pose1.pose.orientation.w = 0.7071
    goal_pose2 = PoseStamped() # [2.0, -7.0, -1.57]
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 2.0
    goal_pose2.pose.position.y = -7.0
    goal_pose2.pose.position.z = 0.0
    goal_pose2.pose.orientation.z = -0.7071
    goal_pose2.pose.orientation.w = 0.7071
    goal_pose3 = PoseStamped() # [-3.0, 2.5, 1.57]
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = -3.0
    goal_pose3.pose.position.y = 2.5
    goal_pose3.pose.orientation.w = 0.7071
    goal_pose3.pose.orientation.z = 0.7071

    
    def feedback():
        
        i=0
        
        while not navigator.isTaskComplete():
            
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.cancelTask()

                

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
        
    navigator.goToPose(goal_pose1)
    feedback()
    print("Successfully travelled to point one")
    navigator.goToPose(goal_pose2)
    feedback()
    print("Successfully travelled to point two")
    navigator.goToPose(goal_pose3)
    feedback()
    print("Successfully travelled to point three")

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()