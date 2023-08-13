#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32MultiArray

rospy.init_node('pos_publisher')


s_pub = rospy.Publisher('/start_position', PoseStamped, queue_size=1)
g_pub = rospy.Publisher('/goal_position', PoseStamped, queue_size=1)

s = PoseStamped()
s.pose.position.x = s.pose.position.y = 15

g = PoseStamped()
g.pose.position.x = 13
g.pose.position.y = 15

x = Int32MultiArray()


print(f"will published {s} to {g}")
while not rospy.is_shutdown():
    s_pub.publish(s)
    g_pub.publish(g)
    x.data = [(1,2),(2,3)]
    # print(x)
