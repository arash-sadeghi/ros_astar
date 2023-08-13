#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

rospy.init_node('pos_publisher')


s_pub = rospy.Publisher('/start_position', Point, queue_size=1)
g_pub = rospy.Publisher('/goal_position', Point, queue_size=1)

s = Point()
s.x = s.y = 15

g = Point()
g.x = 13
g.y = 15

while not rospy.is_shutdown():
    s_pub.publish(s)
    g_pub.publish(g)

