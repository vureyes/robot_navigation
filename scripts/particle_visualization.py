#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np

start_publisher = None
map_img = None
final_img = None
map_loaded = False
particles = []

def visualization_init():
    rospy.init_node('particle_visualization', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, accion_map_cb)
    rospy.Subscriber("/particles", PoseArray, accion_particles_cb)
    plot_map()
    rospy.spin()

def accion_map_cb(occ_grid):
    global map_img
    global map_loaded
    global final_img
    map_loaded = False
    width = occ_grid.info.width
    height = occ_grid.info.height
    map_img = np.array(occ_grid.data).reshape((height, width))
    final_img = np.copy(map_img)
    map_loaded = True


def accion_particles_cb(pose_array):
    global particles
    particles = pose_array.poses
    if map_loaded:
        update_map_particles()

def update_map_particles():
    global final_img
    final_img = np.copy(map_img)
    positions = []
    for pose in particles:
        final_img[int(pose.position.x),int(pose.position.y)] = 50
        positions.append([pose.position.x, pose.position.y])
    position_mean = np.mean(positions, axis=0)
    final_img[int(position_mean[0]), int(position_mean[1])] = 250

def plot_map():
    while True:
        if map_loaded:
            img = np.uint8(final_img)
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            cv2.imshow("Particles", img)
        cv2.waitKey(10)


if __name__ == '__main__':
    try:
        visualization_init()
    except rospy.ROSInterruptException:
        pass