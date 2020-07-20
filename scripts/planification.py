#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np
from helpers import get_theta

path_publisher = None
initial_pose_publisher = None


def planification_init():
    rospy.init_node('path_planification', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, accion_map_cb)
    rospy.Subscriber("/path_completed", Bool, accion_completed_cb)

    global path_publisher
    path_publisher = rospy.Publisher("/path_poses", PoseArray, queue_size=10)
    global initial_pose_publisher
    initial_pose_publisher = rospy.Publisher("/initial_position", Pose, queue_size=10)

    raw_input("Type anything to start > ")
    # Envido de pose inicial hardcodeado
    # Esto se deberia cargar desde el yaml
    pose = Pose()
    pose.position.x = 2
    pose.position.y = 2
    initial_pose_publisher.publish(pose)

    rospy.spin()


def accion_map_cb(occ_grid):
    # global map_img
    # global map_loaded
    # global final_img
    # map_loaded = False
    # width = occ_grid.info.width
    # height = occ_grid.info.height
    # map_img = np.array(occ_grid.data).reshape((height, width))
    # final_img = np.copy(map_img)
    # map_loaded = True

    # Un ejemplo de lo que hace el recibir mapa en los otros nodos, deberia ser algo parecido
    # Probablemente sea lo mismo para recibir la imagen
    # Idealmente cuando se recibe el mapa se deberia generar el grafo de conectividad
    pass

def accion_completed_cb(boolean):
    # Recibe un mensaje desde el nodo de movimiento que le dice que ya termino de avanzar
    # hasta la ultima pose de la lista
    # Deberia sacar una pose de la lista de objetivos y hacer trigger de la generacion de un camino
    # para llegar a ese objetivo
    # (Probablemente la espera de 5 segundos deberia ser aqui)
    # (Tambien deberia revisar aqui si es destino intermedio o llego al final y mandar signal a nodo de sonido)
    pass


if __name__ == '__main__':
    try:
        planification_init()
    except rospy.ROSInterruptException:
        pass