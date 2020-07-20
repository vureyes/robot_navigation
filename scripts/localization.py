#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
from localization_tools import ParticleLocalization
from helpers import get_theta, get_quaternion

laser_reading = None
navigation_publisher = None

N_PARTICLES = 1000

execution_started = False
current_pose = None
last_pose = Pose()

particle_localization = None
particle_publisher = None
movement_publisher = None
pose_publisher = None
sound_publisher = None
complete_publisher = None

def localization_init():
    rospy.init_node('localization', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, accion_map_cb)
    rospy.Subscriber("/scan", LaserScan, accion_laser_cb)
    rospy.Subscriber("/odom", Odometry, accion_odom_cb)
    rospy.Subscriber("/command", String, accion_command_cb)
    rospy.Subscriber("/initial_position", Pose, accion_initial_pose_cb)

    global particle_publisher
    particle_publisher = rospy.Publisher("/particles", PoseArray, queue_size=10)
    global movement_publisher
    movement_publisher = rospy.Publisher("/move", Bool, queue_size=10)
    global sound_publisher
    sound_publisher = rospy.Publisher("/sound_string", String, queue_size=10)
    global pose_publisher
    pose_publisher = rospy.Publisher("/particle_pose", Pose, queue_size=10)
    global complete_publisher
    complete_publisher = rospy.Publisher("/path_completed", Bool, queue_size=10)
    rospy.spin()

def accion_map_cb(occ_grid):
    width = occ_grid.info.width
    height = occ_grid.info.height
    map_resolution = occ_grid.info.resolution
    map_img = np.array(occ_grid.data).reshape((height, width))
    global particle_localization
    particle_localization = ParticleLocalization(map_img, map_resolution, N_PARTICLES, fixed_angle=False)
    rate = rospy.Rate(0.5)
    rate.sleep()
    # send_particles()

def accion_command_cb(command):
    if command.data == "test":
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
    
def accion_odom_cb(odom):
    global current_pose
    current_pose = odom.pose.pose
    if execution_started:
        run_localization()
        send_particles()

def accion_initial_pose_cb(pose):
    x = pose.position.x
    y = pose.position.y
    theta = get_theta(pose.orientation)
    particle_localization.create_particles_from_pose([x,y,theta])
    run_localization()
    send_particles()
    determine_location()
    global execution_started
    execution_started = True
    complete_publisher.publish(Bool(True))
    
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
    average = np.mean(particles, axis=0)
    pose = Pose()
    pose.position.x = average[0]
    pose.position.y = average[1]
    quat = get_quaternion(average[2])
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    pose_publisher.publish(pose)
    particle_publisher.publish(msg)


def determine_location():
    location_found = particle_localization.fit_particles()
    if not location_found:
        sound_publisher.publish(String("lost"))
    else:
        sound_publisher.publish(String("located"))
    
    # movement_publisher.publish(Bool())


if __name__ == '__main__':
    try:
        localization_init()
    except rospy.ROSInterruptException:
        pass