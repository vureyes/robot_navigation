#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np
from helpers import get_theta, get_quaternion
import yaml

path_publisher = None
initial_pose_publisher = None
pose_list = []
map_img = None
map_resolution = None
last_pose = None


def planification_init():
    rospy.init_node('path_planification', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, accion_map_cb)
    rospy.Subscriber("/path_completed", Bool, accion_completed_cb)

    global path_publisher
    path_publisher = rospy.Publisher("/path_poses", PoseArray, queue_size=10)
    global initial_pose_publisher
    initial_pose_publisher = rospy.Publisher("/initial_position", Pose, queue_size=10)

    raw_input("Type anything to start > ")
    import_poses()

    rospy.spin()

def import_poses():
    file = open("poses.yaml")
    document = yaml.load(file)
    pose1 = document["pose1"]
    pose = build_pose(pose1[0], pose1[1], pose1[2])
    initial_pose_publisher.publish(pose)
    pose_list.append(document["pose2"])
    pose_list.append(document["pose3"])
    global last_pose
    last_pose = pose1

def accion_map_cb(occ_grid):
    global map_img
    global map_loaded
    global map_resolution
    map_loaded = False
    width = occ_grid.info.width
    height = occ_grid.info.height
    map_resolution = occ_grid.info.resolution
    map_img = np.array(occ_grid.data).reshape((height, width))
    map_loaded = True

    # Un ejemplo de lo que hace el recibir mapa en los otros nodos, deberia ser algo parecido
    # Probablemente sea lo mismo para recibir la imagen
    # Idealmente cuando se recibe el mapa se deberia generar el grafo de conectividad
    pass

def accion_completed_cb(boolean):
    if len(pose_list) > 0:
        send_poses([pose_list.pop(0)], pixels = False)
    else:
        print("Terminado!!")
    # Recibe un mensaje desde el nodo de movimiento que le dice que ya termino de avanzar
    # hasta la ultima pose de la lista
    # Deberia sacar una pose de la lista de objetivos y hacer trigger de la generacion de un camino
    # para llegar a ese objetivo
    # (Probablemente la espera de 5 segundos deberia ser aqui)
    # (Tambien deberia revisar aqui si es destino intermedio o llego al final y mandar signal a nodo de sonido)
    pass

def send_poses(poses, pixels=True):
    pose_array = PoseArray()
    for p in poses:
        pose_x = p[0]
        pose_y = p[1]
        pose_theta = p[2]
        if not pixels:
            pose_y = p[0]/map_resolution
            pose_x = map_img.shape[0] - p[1]/map_resolution
            pose_theta = pose_theta + np.pi/2
        print(pose_x, pose_y, pose_theta)
        pose = build_pose(pose_x,pose_y,pose_theta)
        pose_array.poses.append(pose)
    path_publisher.publish(pose_array)

def build_pose(x,y,theta):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    quat = get_quaternion(theta)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose


if __name__ == '__main__':
    try:
        planification_init()
    except rospy.ROSInterruptException:
        pass