#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped

rospy.init_node('pos_publisher')


s_pub = rospy.Publisher('/start_position', PoseStamped, queue_size=1)
g_pub = rospy.Publisher('/goal_position', PoseStamped, queue_size=1)

s = PoseStamped()
s.pose.position.x = s.pose.position.y = 15

g = PoseStamped()
g.pose.position.x = 13
g.pose.position.y = 15

while not rospy.is_shutdown():
    s_pub.publish(s)
    g_pub.publish(g)
    print(f"published {s} to {g}")

