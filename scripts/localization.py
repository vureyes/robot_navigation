#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Bool, String
from geometry_msgs.msg import Twist, Pose, PoseArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np
from localization_tools import ParticleLocalization
from movement import get_theta

laser_reading = None
navigation_publisher = None
orientation_enable_publisher=None
orientation_set_point_publisher=None
orientation_plant_state_publisher=None
left_distance = None
front_distance = None

last_linear_vel = 0
last_angluar_vel = 0
control_rate = 10
n_particles = 5000

obstacle_ahead = False
current_pose = None
last_pose = Pose()

particle_localization = None
particle_publisher = None
movement_publisher = None
sound_publisher = None

def localization_init():
    rospy.init_node('localization', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, accion_map_cb)
    rospy.Subscriber("/scan", LaserScan, accion_laser_cb)
    rospy.Subscriber("/odom", Odometry, accion_odom_cb)
    rospy.Subscriber("/command", String, accion_command_cb)

    global particle_publisher
    particle_publisher = rospy.Publisher("/particles", PoseArray, queue_size=10)
    global movement_publisher
    movement_publisher = rospy.Publisher("/move", Bool, queue_size=10)
    global sound_publisher
    sound_publisher = rospy.Publisher("/sound_string", String, queue_size=10)

    rospy.spin()

def accion_map_cb(occ_grid):
    width = occ_grid.info.width
    height = occ_grid.info.height
    map_resolution = occ_grid.info.resolution
    map_img = np.array(occ_grid.data).reshape((height, width))
    global particle_localization
    particle_localization = ParticleLocalization(map_img, map_resolution, n_particles, fixed_angle=False)
    rate = rospy.Rate(0.5)
    rate.sleep()
    send_particles()

def accion_command_cb(command):
    if command.data == "test":
        # particle_localization.test_sensor_model(laser_reading)
        particle_localization.test_MCL(laser_reading)
        send_particles()
    elif command.data == "run":
        run_localization()
        send_particles()
        determine_location()
        # movement_publisher.publish(Bool(True))

    elif command.data == "move":
        particle_localization.test_motion_model([0.2,0,0])
        send_particles()

def accion_laser_cb(data):
    ranges = data.ranges[62:119]
    global laser_reading
    laser_reading = ranges
    global left_distance
    left_distance = ranges[-1]
    global obstacle_ahead
    obstacle_ahead = True if ranges[28]< 0.45 or ranges[-1]<0.45 else False


def accion_odom_cb(odom):               # Callback que se encarga de actualizar posicion basada en odometria
    global current_pose
    current_pose = odom.pose.pose
    

def run_localization():
    global last_pose
    dist_x = current_pose.position.x - last_pose.position.x
    dist_y = current_pose.position.y - last_pose.position.y
    dist_ang = get_theta(current_pose.orientation) - get_theta(last_pose.orientation)
    movement = [dist_x, dist_y, dist_ang]
    last_pose = current_pose
    particle_localization.run_MCL(movement, laser_reading)
    particle_localization.fit_particles()

def send_particles():
    particles = particle_localization.particles
    msg = PoseArray()
    for particle in particles:
        pose = Pose()
        pose.position.x = particle[0]
        pose.position.y = particle[1]
        pose.orientation.z = particle[2]
        msg.poses.append(pose)
    particle_publisher.publish(msg)

def determine_location():
    location_found = particle_localization.fit_particles()
    if not location_found:
        sound_publisher.publish(String("lost"))
    else:
        sound_publisher.publish(String("located"))
    
    movement_publisher.publish(Bool())


if __name__ == '__main__':
    try:
        localization_init()
    except rospy.ROSInterruptException:
        pass