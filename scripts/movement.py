#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Bool, String
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np
from localization_tools import ParticleLocalization
from math import atan2, pi, sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler

MOVEMENT_DISTANCE = 50

# Publishers de controlador de rotacion
rotation_enable_publisher = None
rotation_set_point_publisher = None
rotation_plant_state_publisher = None
rotation_control_effort = 0
# Variables de controlador de rotacion
current_rotation_goal = 0
last_rotation_effort = 0
rotation_pid_lock = False
rotation_starting_point = None


# Publishers de controlador de rotacion
distance_enable_publisher = None
distance_set_point_publisher = None
distance_plant_state_publisher = None
distance_control_effort = 0
# Variables de controlador de rotacion
current_distance_goal = 0
last_distance_effort = 0
distance_pid_lock = False
distance_starting_point = None

pid_rate = 50   #Hz
current_pose = None
navigation_publisher=None

laser_reading = None
left_distance = None
obstacle_ahead = None

command_publisher = None

def movement_init():
    rospy.init_node('movement', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, accion_laser_cb)
    # rospy.Subscriber("/odom", Odometry, accion_odom_cb)
    rospy.Subscriber("/move", Bool, accion_move_cb)
    rospy.Subscriber("/particle_pose", Pose, accion_pose_cb)

    global navigation_publisher
    navigation_publisher = rospy.Publisher('/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=10)
    global command_publisher
    command_publisher= rospy.Publisher('/command', String, queue_size=10)

    #Topicos de controlador de rotacion
    rospy.Subscriber("/robot_rotation/control_effort", Float64, accion_rotation_control_effort_cb)
    global rotation_set_point_publisher
    rotation_set_point_publisher = rospy.Publisher("/robot_rotation/setpoint", Float64, queue_size=10)
    global rotation_plant_state_publisher
    rotation_plant_state_publisher = rospy.Publisher("/robot_rotation/state", Float64, queue_size=10)
    global rotation_enable_publisher
    rotation_enable_publisher = rospy.Publisher("/robot_rotation/pid_enable", Bool, queue_size=10)

    #Topicos de controlador de distancia
    rospy.Subscriber("/robot_distance/control_effort", Float64, accion_distance_control_effort_cb)
    global distance_set_point_publisher
    distance_set_point_publisher = rospy.Publisher("/robot_distance/setpoint", Float64, queue_size=10)
    global distance_plant_state_publisher
    distance_plant_state_publisher = rospy.Publisher("/robot_distance/state", Float64, queue_size=10)
    global distance_enable_publisher
    distance_enable_publisher = rospy.Publisher("/robot_distance/pid_enable", Bool, queue_size=10)

    # run_prompt()

    rospy.spin()

def accion_rotation_control_effort_cb(rotation_effort):
    ejecutar_giro(rotation_effort.data)

def accion_distance_control_effort_cb(distance_effort):
    print(distance_effort.data)
    ejecutar_desplazamineto(distance_effort.data)

def accion_laser_cb(data):
    ranges = data.ranges[62:119]
    global laser_reading
    laser_reading = ranges
    global left_distance
    left_distance = ranges[-1]
    global obstacle_ahead
    obstacle_ahead = True if min(ranges) < 0.35 else False
    # obstacle_ahead = True if ranges[28]< 0.45 or ranges[-1]<0.45 else False

def accion_odom_cb(odom):               # Callback que se encarga de actualizar posicion basada en odometria
    global current_pose
    current_pose = odom.pose.pose

def accion_pose_cb(pose):
    print(pose)
    global current_pose
    current_pose = pose

def accion_move_cb(boolean):
    if obstacle_ahead:
        angle = get_theta(current_pose.orientation)
        giro_controlado(angle + pi/4)
    else:
        goal = set_goal()
        desplazamiento_controlado(goal)

def set_goal():
    angle = get_theta(current_pose.orientation)
    x = current_pose.position.x + MOVEMENT_DISTANCE*np.cos(angle)
    y = current_pose.position.y + MOVEMENT_DISTANCE*np.sin(angle)
    goal = Pose()
    goal.position.x = x
    goal.position.y = y
    goal.orientation.z = angle
    return goal

def notify_movement_end():
    command_publisher.publish(String("run"))

def run_prompt():
    while True:
        command = raw_input("> ")
        msg = String(command)
        command_publisher.publish(msg)

'''
#####################################
#          Giro controlado          #
#####################################
'''
def giro_controlado(goal_theta):
    set_rotation_lock(True)                 # Tomamos el lock del pid de rotacion
    rotation_set_point(goal_theta)          # Seteamos el punto de referencia
    rotation_control_enable(True)           # Habilitamos el pid para funcionar
    rate = rospy.Rate(pid_rate)
    while rotation_pid_lock:                # Mientras se mantenga el lock de rotacion
        rotation_publish_state()            # Enviamos el estado de planta al controlador
        rate.sleep()
    print("Giro Finalizado")
    notify_movement_end()

def ejecutar_giro(rotation_effort):
    # print(current_rotation_goal, get_theta(current_pose.orientation))
    actual_rotation = diferencia_angulos(get_theta(current_pose.orientation), get_theta(rotation_starting_point.orientation))
    # Si el error es lo suficientemente chico y el esfuerzo del controlador tambien, detenemos el avance
    if abs(current_rotation_goal -  actual_rotation) < 0.005 and abs(rotation_effort) < 0.05:
        set_rotation_lock(False)
        rotation_control_enable(False)
        rotation_effort = 0
    # Sino, procedemos a saturar
    global last_rotation_effort
    rotation_effort = saturate(rotation_effort, last_meassure=last_rotation_effort, max_value=0.5, rate=pid_rate)
    # Y enviamos la velocidad al topico
    velocidad = Twist()
    velocidad.angular.z = rotation_effort
    navigation_publisher.publish(velocidad)
    last_rotation_effort = rotation_effort
    
def rotation_control_enable(boolean):
    msg = Bool()
    msg.data = boolean
    rotation_enable_publisher.publish(msg)

def rotation_publish_state():
    msg = Float64()
    msg.data = diferencia_angulos(get_theta(current_pose.orientation),get_theta(rotation_starting_point.orientation))
    # msg.data = get_theta(current_pose.orientation) - get_theta(rotation_starting_point.orientation)
    rotation_plant_state_publisher.publish(msg)
    # print("Plant state",msg.data)

def rotation_set_point(theta):
    global current_rotation_goal
    global rotation_starting_point
    angle = diferencia_angulos(theta, get_theta(current_pose.orientation))
    msg = Float64()
    msg.data = angle
    rotation_set_point_publisher.publish(msg)
    current_rotation_goal = angle
    rotation_starting_point = current_pose

def set_rotation_lock(boolean):
    global rotation_pid_lock
    rotation_pid_lock = boolean

'''
#####################################
#     Desplazamiento controlado     #
#####################################
'''
def desplazamiento_controlado(goal):
    set_distance_lock(True)
    distance_set_point(goal)          # Seteamos el punto de referencia
    distance_control_enable(True)           # Habilitamos el pid para funcionar
    rate = rospy.Rate(pid_rate)
    while distance_pid_lock:                # Mientras se mantenga el lock de rotacion
        distance_publish_state()            # Enviamos el estado de planta al controlador
        rate.sleep()
    print("Desplazamiento Finalizado")
    notify_movement_end()

def ejecutar_desplazamineto(distance_effort):
    global last_distance_effort
    # Si el error es lo suficientemente chico y el esfuerzo del controlador tambien, detenemos el avance
    if abs(current_distance_goal - distance(distance_starting_point,current_pose)) < 10 and abs(distance_effort) < 0.05:
        set_distance_lock(False)
        distance_control_enable(False)
        distance_effort = 0
    elif obstacle_ahead:
        if last_distance_effort == 0:
            set_distance_lock(False)
            distance_control_enable(False)
        distance_effort = 0
    # Saturar el esfuerzo del controlador
    distance_effort = saturate(distance_effort, last_meassure=last_distance_effort, max_value=0.5, rate=pid_rate)
    # Aplicar el esfuerzo
    velocidad = Twist()
    velocidad.linear.x = distance_effort
    navigation_publisher.publish(velocidad)
    last_distance_effort = distance_effort
    # print("Distance effort:",distance_effort)

def distance_control_enable(boolean):
    msg = Bool()
    msg.data = boolean
    distance_enable_publisher.publish(msg)

def distance_publish_state():
    dist = distance(distance_starting_point, current_pose)
    msg = Float64()
    msg.data = dist
    distance_plant_state_publisher.publish(dist)
    # print("State",dist)

def distance_set_point(goal):
    global current_distance_goal
    global distance_starting_point
    dist = distance(goal, current_pose)
    msg = Float64()
    msg.data = dist
    distance_set_point_publisher.publish(msg)
    current_distance_goal = dist
    distance_starting_point = current_pose
    # print("Meta:", current_distance_goal)

def set_distance_lock(boolean):
    global distance_pid_lock
    distance_pid_lock = boolean

'''
#####################################
#     Procesamiento de Goals        #
#####################################
'''

def get_theta(quaternion):
    theta = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])[2]
    if theta < 0:
        theta = theta + 2*pi
    elif theta > 2*pi:
        theta = theta%(2*pi)
    return theta

def get_quaternion(theta):
    quaternion = quaternion_from_euler(0,0,theta,'xyz')
    return quaternion

# Calcula primer angulo para apuntar al objetivo
def angulo_direccion(goal_y, current_y, goal_x, current_x):
    angulo = atan2(goal_y-current_y, goal_x - current_x)
    return angulo

# Calcula la diferencia entre dos angulos entregando giro mas corto
def diferencia_angulos(angle1, angle2):
    angle = angle1-angle2
    if angle > pi: return angle - 2*pi
    elif angle < -pi: return angle + 2*pi
    return angle

def distance(pose1, pose2):
    dist = abs((pose2.position.y-pose1.position.y)**2 + (pose2.position.x-pose1.position.x)**2)
    return dist

'''
#####################################
#            Saturacion             #
#####################################
'''
def saturate(signal, last_meassure, max_value, rate):
    if (signal - last_meassure)*rate > max_value:
        return last_meassure + max_value/rate
    elif (signal - last_meassure)*rate < -max_value:
        return last_meassure - max_value/rate
    return signal

if __name__ == '__main__':
    try:
        movement_init()
    except rospy.ROSInterruptException:
        pass