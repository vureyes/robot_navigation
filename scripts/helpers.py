import cv2 as cv
from cv_bridge import CvBridge
import numpy as np
from math import atan2, pi, sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def get_theta(quaternion):
    theta = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])[2]
    if theta < 0:
        theta = theta + 2*pi
    elif theta > 2*pi:
        theta = theta%(2*pi)
    return theta

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