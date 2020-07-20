#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid

class Node:

    def __init__(self, value):
        self.value = value
        self.parent = None
        self.neighbours = {
            "up": None,
            "down": None,
            "left": None,
            "right": None
        }

class MapPath:

    def __init__(self):
        rospy.init_node('path_bfs', anonymous=True)
        rospy.Subscriber("/map", OccupancyGrid, self.accion_map_cb)
        self.map_img = np.array([])
        self.map_resolution = 0
        self.map_loaded = False
        self.graph = []
        rospy.spin()

    def accion_map_cb(self, occ_grid):
        self.map_loaded = False
        width = occ_grid.info.width
        height = occ_grid.info.height
        self.map_resolution = occ_grid.info.resolution
        self.map_img = np.array(occ_grid.data).reshape((height, width))
        self.graph = []
        self.map_loaded = True
        self.map_to_graph()
        path = self.bfs(12, 5)
        poses = self.path_to_pose(path)

    def map_to_graph(self):
        for i in range(18):
            self.graph.append(Node(i))
        for i in range(18):
            value = i
            node = self.search_node(i)
            #rospy.loginfo(node)
            x, y = self.node_to_pixel(node)
            up = value - 6
            down = value + 6
            left = value - 1
            right = value + 1
            if down <= 17:
                if self.map_img[y + 16, x] != 100:
                    node.neighbours['down'] = self.search_node(down)
                    #rospy.loginfo("down: {}".format(down))
            if up >= 0:
                if self.map_img[y -16, x] != 100:
                    node.neighbours['up'] = self.search_node(up)
                    #rospy.loginfo("up: {}".format(up))
            if left >= 0 and value % 6 != 0:
                if self.map_img[y, x - 16] != 100:
                    node.neighbours['left'] = self.search_node(left)
                    #rospy.loginfo("left: {}".format(left))
            if right <= 17 and value % 6 != 5:
                if self.map_img[y, x +16] != 100:
                    node.neighbours['right'] = self.search_node(right)
                    #rospy.loginfo("right: {}".format(right))
    
    def node_to_pixel(self, node):
        value = node.value
        x = value % 6
        y = value // 6
        x_cord = 32 * x + 16
        y_cord = 32 * y + 16
        return x_cord, y_cord
    
    def path_to_pose(self, path):
        poses = []
        for i in range(len(path)-1):
            value = path[i]
            x_pix, y_pix = self.node_to_pixel(self.search_node(value))
            dif_value = path[i+1] - path[i]
            angle = 0
            if dif_value == -6:
                angle = np.pi / 2
            elif dif_value == 6:
                angle = 3 * np.pi /2
            elif dif_value == 1:
                angle = 0
            elif dif_value == -1:
                angle = np.pi
            pose = [x_pix, y_pix, angle]
            poses.append(pose)
        value = path[len(path) -1]
        x_pix, y_pix = self.node_to_pixel(self.search_node(value))
        pose = [x_pix, y_pix, angle]
        poses.append(pose)
        return poses
    
    def node_to_pose(self, node):
        pass

    def pixel_to_node(self, pose):
        x = pose.position.x // 32
        y = pose.position.y // 32
        value = y * 6 + x
        return value

    def bfs(self, initial_value, goal_value):
        initial = self.search_node(initial_value)
        goal = self.search_node(goal_value)
        open = []
        discovered = [initial]
        while initial.value != goal.value:
            rospy.loginfo(initial.value)
            for node in initial.neighbours.values():
                if node:
                    if node not in discovered:
                        node.parent = initial
                        open.append(node)
                        discovered.append(node)
            if len(open) == 0:
                return None
            initial = open.pop(0)
        path = []
        while initial:
            path.append(initial.value)
            initial = initial.parent
        path.reverse()
        return path

    def search_node(self, value):
        if value < 0 or value > 17:
            return
        node = None
        for x in self.graph:
            if x.value == value:
                return x
        return node

if __name__ == "__main__":
    m = MapPath()
