#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
import numpy as np

def map_callback(msg):
    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution
    map_array = np.array(msg.data)
    
    map_array[300 * width + 300] = 200
    
    map_data = map_array.reshape((height, width))
    
    plt.imshow(map_data, cmap='gray', origin='lower', extent=(0, width*resolution, 0, height*resolution))
    plt.colorbar()
    plt.title('Occupancy Grid Map')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()

def main():
    rospy.init_node('map_visualizer', anonymous=True)
    
    map_topic = "/map"  # Change this to the actual map topic
    
    rospy.Subscriber(map_topic, OccupancyGrid, map_callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
