#!/usr/bin/env python3

# PUBLICACION DE POSE DE SISTEMA DE ARUCO CON POSE ARRAY

# MI MAGNUM OPUS

from rclpy.node import Node
from scipy.spatial.transform import Rotation
import transforms3d
import numpy as np   
import cv2
import math

from aru_interfaces.msg import ArucoMarker, ArucoList
from collections import defaultdict

from .POS_CAL import CAM
from .POS_FU import POS
from .POS_IM import POINTS
from .POS_ARU import ARU

class ObjPos(Node):

    def __init__(self):
        super().__init__('ObjPos')
        self.get_logger().info('DETECCION DE CAJAS')

        # PUBLISHER PARA POSE DE WAYPOINT EN EL MUNDO
        self.pos_pub = self.create_publisher(ArucoList, '/pzb/aru_sys', 10)
       
        # MENSAJE DE POSE
        self.poseArr = ArucoList()
        self.poseW = ArucoMarker()

        # CALIBRACION DE CAMARA
        self.cam = CAM('/world22.png')
        self.cam.cameraCalibrate()

        # PARA DETECCION Y TRACKING DE ARUCO
        self.aru = ARU()

        # POSICION DE MUNDO Y CAMARA
        self.pos = POS(self.cam)
        self.Rc, self.Pc = self.pos.worldCal()

        # ARUCO LIST
        self.aruList = 0

        # POSICION DE ARUCO EN CAMARA
        self.aruPOS = np.zeros((3,1),float)
        # MATRIZ DE ROTACION DE ARUCO EN CAMARA
        self.aruORI = np.zeros((3,3),float)

        # ANGULOS EULER DE ROTACION DE ARUCO EN EL MUNDO 
        self.aruEULER = np.zeros((3,1),float)
        # ANGULOS EULER DE ROTACION CAMARA PZ
        self.pzEULER = np.zeros((3,1),float)        

        # DISTANCIA DEL CENTRO DEL ARUCO AL WAYPOINT GENERADO (MM)
        self.wayPointD = 100

        # QUATERNIONS FOR CAMERA AND REF
        self.quatC = 0
        self.quatR = 0

        # FOR VISUALIZATION IN RVIZ
        self.scale = 0.001

        # PARA MOVING AVERAGE (HORIZONTE = 10)
        self.aruId = defaultdict(lambda: POINTS(10))

        self.show()


    def show(self):

        IMG = cv2.VideoCapture(0)

        # CALCULAR POSE DE CAMARA Y MUNDO
        self.calCamSys()
        # SETUP ARUCO DETECTOR
        self.aru.detectorSetup()
        
        while True:

            ret, frame = IMG.read()
            
            if(ret is None):
                continue
        
            # ARU DISTANCIA
            frame, self.aruList  = self.aru.pose_estimation(frame,self.cam.CamMatN,self.cam.CamDis)
            # self.get_logger().info('A : '+str(self.aruList))

            for i in range(len(self.aruList)):
                if(not(self.aruList[i][2] == -1)):
                    # CALCULAR MATRIZ DE ROTACION DE ARUCO EN CAMARA
                    self.aruORI = cv2.Rodrigues(self.aruList[i][0])[0]
                    # CALCULAR POSICION DE ARUCO EN CAMARA
                    self.aruPOS = self.aruList[i][1]
                    self.pubPosArray(self.scale,i)
                else:  
                    # **LIMPIAR DEFAULT DICT DE MA** 
                    # SOLO ESTOY BORRANDO SI NO SE ENCUENTRA NINGUN ARUCO.
                    # QUIZAS PUEDA CAUSAR PROBLEMAS, SI DEJA DE DETECTAR
                    # UNO ENTRE MUCHOS SU ID SE MANTIENE EN EL DICCIONARIO. 
                    self.aruId.clear()
            
            self.pos_pub.publish(self.poseArr)
            self.poseArr.markers[1:] = []

            cv2.imshow("Imagen",frame)

            k = cv2.waitKey(1)

            if(k == ord('q')):
                break

        IMG.release()
        cv2.destroyAllWindows()

    
    def pubPosArray(self,scale,i):
        self.poseArr.header.stamp = self.get_clock().now().to_msg()
        self.poseArr.header.frame_id = 'base_link'

        poses = ArucoMarker()

        # CAM -> ARU CENTER
        # ---------------------------------------
        poses.id = int(self.aruList[i][2]) # ENTRAR AL MOVING AVERAGE CORRESPONDIENTE A CADA ID
        poses.pose.position.x = self.aruId[poses.id].ma_get(self.aruPOS[0][0],0)*scale
        poses.pose.position.y = self.aruId[poses.id].ma_get(self.aruPOS[1][0],1)*scale
        poses.pose.position.z = self.aruId[poses.id].ma_get(self.aruPOS[2][0],2)*scale
        
        r = Rotation.from_matrix(self.aruORI)
        self.quatR = r.as_quat()

        quat = self.quatR

        poses.pose.orientation.x = self.aruId[poses.id].ma_get(quat[0],3)*scale
        poses.pose.orientation.y = self.aruId[poses.id].ma_get(quat[1],4)*scale
        poses.pose.orientation.z = self.aruId[poses.id].ma_get(quat[2],5)*scale
        poses.pose.orientation.w = self.aruId[poses.id].ma_get(quat[3],6)*scale
        # ---------------------------------------

        self.poseArr.markers.append(poses)

    
    # ESTABLECER REFERENCIALES DE MUNDO Y CAMARA SOLAMENTE UNA VEZ
    def calCamSys(self):
        # BASE LINK -> CAM
        self.pzEULER = self.matrix2euler(self.Rc)
        self.quatC = transforms3d.euler.euler2quat(self.pzEULER[0]+1.5708, 
                                                   self.pzEULER[1], 
                                                   self.pzEULER[2]-1.5708)  
        # BASE LINK -> CAM
        # ---------------------------------------
        self.poseW.id = -2
        self.poseW.pose.position.x = 0.108
        self.poseW.pose.position.y = 0.0
        self.poseW.pose.position.z = 0.044

        self.poseW.pose.orientation.x = self.quatC[1]
        self.poseW.pose.orientation.y = self.quatC[2]
        self.poseW.pose.orientation.z = self.quatC[3]
        self.poseW.pose.orientation.w = self.quatC[0]
        # --------------------------------------- 

        self.poseArr.markers[:] = [self.poseW]


    # FUNCION PARA CONVERTIR MATRIZ DE ROTACION A ANGULOS DE EULER
    def matrix2euler(self, Rmat):
        sy = math.sqrt(Rmat[0,0] * Rmat[0,0] + Rmat[1,0] * Rmat[1,0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(Rmat[2,1], Rmat[2,2])
            y = math.atan2(-Rmat[2,0], sy)
            z = math.atan2(Rmat[1,0], Rmat[0,0])
        else:
            x = math.atan2(-Rmat[1,2], Rmat[1,1])
            y = math.atan2(-Rmat[2,0], sy)
            z = 0
        return np.array([x, y, z], float)
