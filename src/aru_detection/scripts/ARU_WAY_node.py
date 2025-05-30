#!/usr/bin/env python3

# CALCULO DESDE CERO DE ARUCO CENTER EN ODOM CON MATRICES HOMOGENEAS
# *****CURRENTLY USELESS*****

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np

from aru_interfaces.msg import ArucoList

class AruWay(Node):

    def __init__(self):
        super().__init__('AruWay')
        self.get_logger().info('WAY CALC')
        
        self.pzb_pos = self.create_subscription(Odometry, '/pzb/odom', self.odom_callback, 10)
        self.aru_pos = self.create_subscription(ArucoList, '/pzb/aru_sys', self.aru_sys_callback,10)
        
        self.br = TransformBroadcaster(self)
        self.t = TransformStamped()

        # PARA ROTACION DE CAMARA
        self.Rc = np.zeros((3,3), float)

        # DE BASE AXIS A BASE LINK (MM)
        self.T_axis = np.array([-0.052, 0.0, 0.049])
        # DE BASE LINK A CAMARA
        self.T_base = np.array([0.108, 0.0, 0.044])
        # MATRIZ HOMOGENEA DE AXIS
        self.H_T_axis = self.make_homo(np.eye(3),self.T_axis)

        self.aruPoses = ArucoList()
        
        self.initialized = False

        self.T_cam = []


    def odom_callback(self,msg):

        if(len(self.aruPoses.markers) < 2):
            return 
        

        self.T_odom = np.array([msg.pose.pose.position.x, 
                                msg.pose.pose.position.y, 
                                msg.pose.pose.position.z])
        
        Ro = Rotation.from_quat([msg.pose.pose.orientation.x,
                                 msg.pose.pose.orientation.y,
                                 msg.pose.pose.orientation.z,
                                 msg.pose.pose.orientation.w])
        
        H_T_odom = self.make_homo(Ro.as_matrix(),self.T_odom)
        H_T_cam_odom = H_T_odom @ self.H_T_axis @ self.H_T_base

        for i in range(1,len(self.aruPoses.markers[1:])+1):
            # ROTATION
            # -------------------------------------------------------------------------
            Ra = Rotation.from_quat([self.aruPoses.markers[i].pose.orientation.x,
                                    self.aruPoses.markers[i].pose.orientation.y,
                                    self.aruPoses.markers[i].pose.orientation.z,
                                    self.aruPoses.markers[i].pose.orientation.w])

            quat = Rotation.as_quat(Ro*self.Rc*Ra)
            # -------------------------------------------------------------------------

            # TRANSLATION
            # -------------------------------------------------------------------------
            # MATRIZ HOMOGENEA CAM -> ODOM
            point_cam = H_T_cam_odom @ self.T_cam[i-1]
            # -------------------------------------------------------------------------

            # TRANSFORMADA ODOM -> ARU
            self.t.header.stamp = self.get_clock().now().to_msg()
            self.t.header.frame_id = 'odom'
            self.t.child_frame_id = 'id'+str(self.aruPoses.markers[i].id)

            self.t.transform.translation.x = float(point_cam[0])
            self.t.transform.translation.y = float(point_cam[1])
            self.t.transform.translation.z = float(point_cam[2])

            self.t.transform.rotation.x = quat[0]
            self.t.transform.rotation.y = quat[1]
            self.t.transform.rotation.z = quat[2]
            self.t.transform.rotation.w = quat[3]

            self.br.sendTransform(self.t)


    def aru_sys_callback(self,pos):
        
        self.aruPoses = pos
        self.T_cam.clear()
        
        for i in range(1,len(self.aruPoses.markers[1:])+1):
            # POSICION DE ARUCO EN EL REFERENCIAL DE CAM     
            temp = np.array([[self.aruPoses.markers[i].pose.position.x],
                            [self.aruPoses.markers[i].pose.position.y],
                            [self.aruPoses.markers[i].pose.position.z], 
                            [1.0]])
            self.T_cam.append(temp)

        # SOLAMENTE UNA VEZ
        if(not self.initialized):
            # ROTACION DE CAMARA EN REFERENCIAL DE BASE_LINK
            Rc = Rotation.from_quat([self.aruPoses.markers[0].pose.orientation.x,
                                    self.aruPoses.markers[0].pose.orientation.y,
                                    self.aruPoses.markers[0].pose.orientation.z,
                                    self.aruPoses.markers[0].pose.orientation.w])
            self.Rc = Rc
            # MATRIZ HOMOGENEA DE BASE
            self.H_T_base = self.make_homo(Rc.as_matrix(),self.T_base)
            self.initialized = True


    # CREAR MATRIZ HOMOGENEA
    def make_homo(self,rot,tran):
        H = np.eye(4)
        H[:3, :3] = rot
        H[:3, 3] = tran
        return H


def main(args=None):
    rclpy.init(args=args)
    node = AruWay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()