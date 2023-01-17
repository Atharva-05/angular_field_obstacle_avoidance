#!/usr/bin/env python3	

import rospy
from geometry_msgs.msg import PoseStamped

rospy.init_node('goal_publisher')
rate = rospy.Rate(10)

goal_publisher = rospy.Publisher('/goal_pose', PoseStamped, queue_size=1)
goal_to_publish = PoseStamped()

if __name__ == '__main__':
    # Code goes here
    while not rospy.is_shutdown():
        x = float(input("\nX Coordinate"))
        y = float(input("Y Coordinate"))
        print("Publishing goal: {} {}", x, y)
        goal_to_publish.pose.position.x = x
        goal_to_publish.pose.position.y = y
        goal_publisher.publish(goal_to_publish)
        rate.sleep()        