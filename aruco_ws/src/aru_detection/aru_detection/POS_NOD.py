#!/usr/bin/env python3

# PUBLICACION DE POSE DE SISTEMA DE ARUCO CON POSE ARRAY

from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from scipy.spatial.transform import Rotation
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

        # PUBLISHER PARA POSE DE WAYPOINT EN EL MUNDO
        self.pos_pub = self.create_publisher(PoseArray, '/pzb/aru_sys', 10)
        # MENSAJE DE POSE
        self.poseArr = PoseArray()

        # CALIBRACION DE CAMARA
        self.cam = CAM('/world22.png')
        self.cam.cameraCalibrate()

        # PARA DETECCION Y TRACKING DE ARUCO
        self.aru = ARU()

        # PARA MOVING AVERAGE
        self.point = [POINTS(5) for _ in range(7)]

        # POSICION DE MUNDO Y CAMARA
        self.pos = POS(self.cam)
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

        # BUFFER PARA POSICION
        self.posBuffer = np.zeros((3,1),float)

        # QUATERNIONS FOR CAMERA AND REF
        self.quatC = 0
        self.quatR = 0
        self.camW = 0

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
            frame, aru_ROT, tvec  = self.aru.pose_estimation(frame,self.cam.CamMat,self.cam.CamDis)
            
            # UNDISTORT FRAME
            dst = cv2.undistort(frame, self.cam.CamMat, self.cam.CamDis, None, self.cam.CamMatN)
            x, y, w, h = self.cam.roi
            dst = dst[y:y+h, x:x+w]

            # REVISAR SI ENCONTRO ARUCO
            if(isinstance(tvec, int)):
                aru_POS = self.posBuffer
            else:
                aru_POS = tvec  
            
            # CALCULAR POSICION DE ARUCO EN CAMARA
            self.aruPOS = aru_POS 

            # CALCULAR ROTACION DE ARUCO EN CAMARA
            self.aruORI = cv2.Rodrigues(aru_ROT)[0]

            # SI LA CAMARA PIERDE EL ARUCO QUEDARSE CON LA ULTIMA POSICION
            if(not(aru_POS.all() == 0)):
                self.posBuffer = aru_POS

            # FOR VISUALIZATION IN RVIZ
            scale = 0.001

            self.pubPosArray(scale)

            cv2.imshow("Imagen",frame)

            k = cv2.waitKey(1)

            if(k == ord('q')):
                break

        IMG.release()
        cv2.destroyAllWindows()
    

    # ESTABLECER REFERENCIALES DE MUNDO Y CAMARA SOLAMENTE UNA VEZ
    def calCamSys(self):
        # BASE LINK -> CAM
        self.pzEULER = self.matrix2euler(self.Rc)
        self.quatC = transforms3d.euler.euler2quat(self.pzEULER[0]+1.5708, 
                                                   self.pzEULER[1], 
                                                   self.pzEULER[2]-1.5708)   

    
    def pubPosArray(self,scale):
        self.poseArr.header.stamp = self.get_clock().now().to_msg()
        self.poseArr.header.frame_id = 'base_link'

        poses = [Pose() for _ in range(2)]
        
        # BASE LINK -> CAM
        # ---------------------------------------
        poses[0].position.x = 0.108
        poses[0].position.y = 0.0
        poses[0].position.z = 0.044

        poses[0].orientation.x = self.quatC[1]
        poses[0].orientation.y = self.quatC[2]
        poses[0].orientation.z = self.quatC[3]
        poses[0].orientation.w = self.quatC[0]

        self.poseArr.poses.append(poses[0])
        # ---------------------------------------

        # CAM -> ARU CENTER
        # ---------------------------------------
        poses[1].position.x = self.point[0].lp_get(self.aruPOS[0][0])*scale
        poses[1].position.y = self.point[1].lp_get(self.aruPOS[1][0])*scale
        poses[1].position.z = self.point[2].lp_get(self.aruPOS[2][0])*scale

        r = Rotation.from_matrix(self.aruORI)
        quat = r.as_quat()

        poses[1].orientation.x = self.point[3].lp_get(quat[0])
        poses[1].orientation.y = self.point[4].lp_get(quat[1])
        poses[1].orientation.z = self.point[5].lp_get(quat[2])
        poses[1].orientation.w = self.point[6].lp_get(quat[3])

        self.poseArr.poses.append(poses[1])
        # ---------------------------------------

        self.pos_pub.publish(self.poseArr)
        self.poseArr.poses.clear()
    

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

