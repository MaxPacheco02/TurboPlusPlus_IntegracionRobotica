#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf2_ros import StaticTransformBroadcaster
from tf2_ros import Buffer
from geometry_msgs.msg import TransformStamped
import transforms3d
import numpy as np   
import cv2
import math

from .POS_CAL import CAM
from .POS_FU import POS
from .POS_IM import POINTS
from .POS_ARU import ARU

class ObjPos(Node):

    def __init__(self):
        super().__init__('ObjPos')
        self.get_logger().info('DETECCION DE CAJAS')

        # PUZZLEBOT BROADCASTER (SOLO POR AHORA STATIC)
        self.puzzlebot_br = StaticTransformBroadcaster(self)

        # ARUCO BREADCASTER
        self.aru_br = TransformBroadcaster(self)

        # TRANSFORM STAMPS
        self.pz_t = TransformStamped()
        self.aru_t = TransformStamped()

        # CALIBRACION DE CAMARA
        self.cam = CAM()
        self.cam.cameraCalibrate()

        # PARA DETECCION Y TRACKING DE ARUCO
        self.aru = ARU()

        # PARA OBTENCION DE PUNTOS EN PLANO DE IMAGEN
        self.point = POINTS(self.aru.markerLength)

        # POSICION DE MUNDO Y CAMARA
        # SELECT NUMBER OF FRAME CORRESPONDING TO THE CHOSEN WORLD FRAME 
        self.pos = POS(self.cam, 3)
        self.Rc, self.Pc = self.pos.worldCal()

        # POSICION DE ARUCO EN EL MUNDO
        self.aruPOS = np.zeros((4,1),float)
        # MATRIZ DE ROTACION DE ARUCO EN EL MUNDO
        self.aruORI = np.zeros((3,3),float)

        # ANGULOS EULER DE ROTACION DE ARUCO EN EL MUNDO 
        self.aruEULER = np.zeros((3,1),float)
        # ANGULOS EULER DE ROTACION CAMARA PZ
        self.pzEULER = np.zeros((3,1),float)        

        # DISTANCIA DEL CENTRO DEL ARUCO AL WAYPOINT GENERADO (MM)
        self.wayPointD = 100

        # POSICION DE MUNDO EN COORDENADAS DE CAMARA
        self.camW = np.zeros((4,1), float)

        self.show()


    def show(self):

        IMG = cv2.VideoCapture(0)
        
        while True:

            ret, frame = IMG.read()
            
            if(ret is None):
                continue

            # FRAME UNDISTORTION
            frame, PUV, aru_ROT, tvec  = self.aru.pose_estimation(frame,self.cam.CamMat,self.cam.CamDis)
            dst = cv2.undistort(frame, self.cam.CamMat, self.cam.CamDis, None, self.cam.CamMatN)
            x, y, w, h = self.cam.roi
            dst = dst[y:y+h, x:x+w]

            # REVISAR SI ENCONTRO ARUCO
            if(isinstance(tvec, int)):
                aru_POS = np.zeros((3,1),float)
            else:
                aru_POS = tvec  
            
            # CALCULAR POSICION DE ARUCO EN MUNDO DE CAMARA
            self.aruPOS = self.pos.aruTranToWorld(aru_POS) 

            # CALCULAR ROTACION DE ARUCO EN EL MUNDO DE CAMARA
            self.aruORI = self.pos.aruOriToWorld(aru_ROT)

            # FOR VISUALIZATION IN RVIZ
            scale = 0.001

            # BASE_LINK -> CAMERA
            # -------------------------------------------------------------------------------------------
            self.pz_t.header.stamp = self.get_clock().now().to_msg()
            self.pz_t.header.frame_id = 'base_link'
            self.pz_t.child_frame_id = 'camera' 

            self.pz_t.transform.translation.x = 0.11
            self.pz_t.transform.translation.y = 0.0
            self.pz_t.transform.translation.z = 0.07

            # MATRIZ DE ROTACIÓN A ANGULOS DE EULER  1.5708
            self.pzEULER = self.matrix2euler(self.Rc)
            q = transforms3d.euler.euler2quat(self.pzEULER[0]+1.5708, self.pzEULER[1], self.pzEULER[2]-1.5708)   

            self.pz_t.transform.rotation.x = q[1]
            self.pz_t.transform.rotation.y = q[2]
            self.pz_t.transform.rotation.z = q[3]
            self.pz_t.transform.rotation.w = q[0]

            self.aru_br.sendTransform(self.pz_t)
            # -------------------------------------------------------------------------------------------


            # CAMARA -> REF 
            # -------------------------------------------------------------------------------------------
            self.pz_t.header.stamp = self.get_clock().now().to_msg()
            self.pz_t.header.frame_id = 'camera'
            self.pz_t.child_frame_id = 'ref' 
            
            # CALCULAR POSICION DE REFERENCIAL DE MUNDO EN COORDENADAS DE CAMARA
            self.camW = self.pos.worldToCam(np.array([[0, 0, 0, 1]]).T)

            self.pz_t.transform.translation.x = self.camW[0][0]*scale
            self.pz_t.transform.translation.y = self.camW[1][0]*scale
            self.pz_t.transform.translation.z = self.camW[2][0]*scale

            # MATRIZ DE ROTACIÓN A ANGULOS DE EULER 
            self.pzEULER = self.matrix2euler(np.linalg.inv(self.Rc))
            q = transforms3d.euler.euler2quat(self.pzEULER[0], self.pzEULER[1], self.pzEULER[2])   

            self.pz_t.transform.rotation.x = q[1]
            self.pz_t.transform.rotation.y = q[2]
            self.pz_t.transform.rotation.z = q[3]
            self.pz_t.transform.rotation.w = q[0]

            self.aru_br.sendTransform(self.pz_t)
            # -------------------------------------------------------------------------------------------


            # REF -> CAJA
            # -------------------------------------------------------------------------------------------
            self.aru_t.header.stamp = self.get_clock().now().to_msg()
            self.aru_t.header.frame_id = 'ref'
            self.aru_t.child_frame_id = 'aruco_center' 

            self.aru_t.transform.translation.x = self.aruPOS[0][0]*scale
            self.aru_t.transform.translation.y = self.aruPOS[1][0]*scale
            self.aru_t.transform.translation.z = self.aruPOS[2][0]*scale

            # MATRIZ DE ROTACIÓN A ANGULOS DE EULER 
            self.aruEULER = self.matrix2euler(self.aruORI)
            q = transforms3d.euler.euler2quat(-self.aruEULER[0], -self.aruEULER[1], -self.aruEULER[2])   

            self.aru_t.transform.rotation.x = q[1]
            self.aru_t.transform.rotation.y = q[2]
            self.aru_t.transform.rotation.z = q[3]
            self.aru_t.transform.rotation.w = q[0]

            self.aru_br.sendTransform(self.aru_t)
            # -------------------------------------------------------------------------------------------


            # CAJA -> WAYPOINT ****FOR DEMO PURPOSES ONLY*****
            # -------------------------------------------------------------------------------------------
            self.aru_t.header.stamp = self.get_clock().now().to_msg()
            self.aru_t.header.frame_id = 'aruco_center'
            self.aru_t.child_frame_id = 'waypoint' 

            self.aru_t.transform.translation.x = 0.0*scale
            self.aru_t.transform.translation.y = 0.0*scale
            self.aru_t.transform.translation.z = self.wayPointD*scale

            # MATRIZ DE ROTACIÓN A ANGULOS DE EULER 
            q = transforms3d.euler.euler2quat(0, 0, 0)   

            self.aru_t.transform.rotation.x = q[1]
            self.aru_t.transform.rotation.y = q[2]
            self.aru_t.transform.rotation.z = q[3]
            self.aru_t.transform.rotation.w = q[0]

            self.aru_br.sendTransform(self.aru_t)
            # -------------------------------------------------------------------------------------------

            cv2.imshow("Imagen",frame)

            k = cv2.waitKey(1)

            if(k == ord('q')):
                break

        IMG.release()
        cv2.destroyAllWindows()


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

