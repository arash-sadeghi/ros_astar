#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import Int32MultiArray
import heapq
import pdb 
class AStarNode:

    def __init__(self):
        rospy.init_node('a_star_node')

        self.map = None
        self.start = None
        self.goal = None

        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.start_sub = rospy.Subscriber('/start_position', Point, self.start_callback)
        self.goal_sub = rospy.Subscriber('/goal_position', Point, self.goal_callback)

        self.path_pub = rospy.Publisher('/path', Int32MultiArray, queue_size=1)

        rospy.spin()

    def map_callback(self, msg):
        self.map = msg.data
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution

    def start_callback(self, msg):
        self.start = (int(msg.x / self.resolution), int(msg.y / self.resolution))

    def goal_callback(self, msg):
        self.goal = (int(msg.x / self.resolution), int(msg.y / self.resolution))

        if self.map is not None and self.start is not None:
            path = self.a_star()
            self.publish_path(path)

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def a_star(self):
        open_list = [(0, self.start)]
        came_from = {}

        while open_list:
            current_cost, current = heapq.heappop(open_list)

            if current == self.goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                print(f"path found rom {self.start} to {self.goal}  path: {path}")
                return path[::-1]

            for neighbor in self.get_neighbors(current):
                cost = current_cost + 1
                try:
                    statement = neighbor not in came_from or cost < came_from[neighbor]
                except Exception as E:
                    print("[-]",E) 
                    pdb.set_trace()
                if statement:
                    came_from[neighbor] = current
                    heapq.heappush(open_list, (cost + self.heuristic(neighbor, self.goal), neighbor))

        print(f"path not found from {self.start} to {self.goal} with resolution {self.resolution}")
        return []

    def get_neighbors(self, pos):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                new_x = pos[0] + dx
                new_y = pos[1] + dy
                # pdb.set_trace()
                if 0 <= new_x < self.width and 0 <= new_y < self.height and self.map[new_y * self.width + new_x] == 0:
                    neighbors.append((new_x, new_y))
        return neighbors

    def publish_path(self, path):
        path_msg = Int32MultiArray(data=path)
        self.path_pub.publish(path_msg)

if __name__ == '__main__':
    try:
        AStarNode()
    except rospy.ROSInterruptException:
        pass
