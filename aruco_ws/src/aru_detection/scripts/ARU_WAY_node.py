#!/usr/bin/env python3

# CALCULO DESDE CERO DE ARUCO CENTER EN ODOM CON MATRICES HOMOGENEAS

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster
import transforms3d
from geometry_msgs.msg import TransformStamped
import numpy as np

class AruWay(Node):

    def __init__(self):
        super().__init__('AruWay')
        self.get_logger().info('WAY CALC')
        
        self.pzb_pos = self.create_subscription(Odometry, '/pzb/odom', self.odom_callback, 10)
        self.aru_pos = self.create_subscription(PoseArray, '/pzb/aru_sys', self.aru_sys_callback,10)
        
        self.br = TransformBroadcaster(self)
        self.t = TransformStamped()

        # PARA ROTACION DE CAMARA
        self.Rc = np.zeros((3,3), float)

        # DE BASE AXIS A BASE LINK (MM)
        self.T_axis = np.array([[-0.052, 0.0, 0.049]]).T
        # DE BASE LINK A CAMARA
        self.T_base = np.array([[0.108, 0.0, 0.044]]).T
        # MATRIZ HOMOGENEA DE AXIS
        self.H_T_axis = np.concatenate((np.concatenate((np.eye(3) , self.T_axis), axis=1),[[0,0,0,1]]),axis=0)

        self.aruPoses = PoseArray()

        self.initialized = False


    def odom_callback(self,msg):

        if(len(self.aruPoses.poses) < 2):
            return 

        self.T_odom = np.array([[msg.pose.pose.position.x, 
                                 msg.pose.pose.position.y, 
                                 msg.pose.pose.position.z]]).T
        
        q1 = np.array([msg.pose.pose.orientation.x,
                       msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z,
                       msg.pose.pose.orientation.w])

        q2 = np.array([self.aruPoses.poses[1].orientation.x,
                      self.aruPoses.poses[1].orientation.y,
                      self.aruPoses.poses[1].orientation.z,
                      self.aruPoses.poses[1].orientation.w])
        
        # ROTATION
        # -------------------------------------------------------------------------
        Ro = Rotation.from_quat(q1)
        Ra = Rotation.from_quat(q2)
        
        quat = Rotation.as_quat(Ro*self.Rc*Ra)
        # -------------------------------------------------------------------------

        # TRANSLATION
        # -------------------------------------------------------------------------
        H_T_odom = np.concatenate((np.concatenate((Ro.as_matrix(), self.T_odom), axis=1),[[0,0,0,1]]),axis=0)
        
        # MATRIZ HOMOGENEA CAM -> ODOM
        H_T_cam_odom = H_T_odom @ self.H_T_axis @ self.H_T_base

        point_cam = H_T_cam_odom @ self.T_cam
        # -------------------------------------------------------------------------

        # TRANSFORMADA ODOM -> ARU
        self.t.header.stamp = self.get_clock().now().to_msg()
        self.t.header.frame_id = 'odom'
        self.t.child_frame_id = 'ref'

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

        # POSICION DE ARUCO EN EL REFERENCIAL DE CAM     
        self.T_cam = np.array([[self.aruPoses.poses[1].position.x,
                                self.aruPoses.poses[1].position.y,
                                self.aruPoses.poses[1].position.z, 
                                1.0]]).T
        # SOLAMENTE UNA VEZ
        if(not self.initialized):
            # ROTACION DE CAMARA EN REFERENCIAL DE BASE_LINK
            camRotq = np.array([self.aruPoses.poses[0].orientation.x,
                                self.aruPoses.poses[0].orientation.y,
                                self.aruPoses.poses[0].orientation.z,
                                self.aruPoses.poses[0].orientation.w])
            Rc = Rotation.from_quat(camRotq)
            self.Rc = Rc
            # MATRIZ HOMOGENEA DE BASE
            self.H_T_base = np.concatenate((np.concatenate((Rc.as_matrix(), self.T_base), axis=1), [[0,0,0,1]]),axis=0)
            self.initialized = True


def main(args=None):
    rclpy.init(args=args)
    node = AruWay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()