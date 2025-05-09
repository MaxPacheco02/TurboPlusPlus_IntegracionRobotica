import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from tf2_ros import StaticTransformBroadcaster
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point, Quaternion
import transforms3d
import numpy as np

class FramePublisher(Node):

    def __init__(self):
        super().__init__('joints_publisher')

        # PUBLISHER PARA POSE DE WAYPOINT EN EL MUNDO
        self.pos_pub = self.create_publisher(Pose, 'waypoint_pose_topic', 10)
        # MENSAJE DE POSE
        self.poseWay = Pose()

        # PARA CONOCER POSE DE WAYPOINT EN EL MUNDO
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.poseWay.orientation = Quaternion()
        self.poseWay.position = Point()

        # TF2
        self.br1 = StaticTransformBroadcaster(self)
        self.br2 = StaticTransformBroadcaster(self)

        self.t = TransformStamped()
        self.t2 = TransformStamped()

        # JOINT STATE PUBLISHER
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)

        #TIMER
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_cb)

        # FOR URDF
        self.ctrlJoints = JointState()
        self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()
        self.ctrlJoints.name = ['joint3','joint4']
        self.ctrlJoints.position = [0.0]*2
        self.ctrlJoints.velocity = [0.0]*2


    def timer_cb(self):
        time = self.get_clock().now().nanoseconds/1e9

        # MAP -> ODOM
        # -------------------------------------------------------------------------------------------
        self.t.header.stamp = self.get_clock().now().to_msg()
        self.t.header.frame_id = 'map'
        self.t.child_frame_id = 'odom'

        self.t.transform.translation.x = 1.0
        self.t.transform.translation.y = 1.0
        self.t.transform.translation.z = 0.0

        q = transforms3d.euler.euler2quat(0, 0, 0)   

        self.t.transform.rotation.x = q[1]
        self.t.transform.rotation.y = q[2]
        self.t.transform.rotation.z = q[3]
        self.t.transform.rotation.w = q[0]

        self.br2.sendTransform(self.t)
        # -------------------------------------------------------------------------------------------


        # ODOM -> FOOTPRINT
        # -------------------------------------------------------------------------------------------
        self.t.header.stamp = self.get_clock().now().to_msg()
        self.t.header.frame_id = 'odom'
        self.t.child_frame_id = 'base_footprint'

        self.t.transform.translation.x = 2.0#2*np.sin(0.1*time)
        self.t.transform.translation.y = 2.0#2*np.cos(0.1*time)
        self.t.transform.translation.z = 0.0

        q = transforms3d.euler.euler2quat(0, 0, 0) #-0.1*time en z   

        self.t.transform.rotation.x = q[1]
        self.t.transform.rotation.y = q[2]
        self.t.transform.rotation.z = q[3]
        self.t.transform.rotation.w = q[0]

        self.br1.sendTransform(self.t)
        # -------------------------------------------------------------------------------------------

        # FRAMES CON URDF (BASE_FOOTPRINT -> BASE_LINK -> ETC..) ROBOT
        self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()
        self.ctrlJoints.position[0] = 2.5*time
        self.ctrlJoints.position[1] = 3*time

        self.publisher.publish(self.ctrlJoints)

        try:
            if self.tf_buffer.can_transform('odom','waypoint',rclpy.time.Time()):
                # TRANSFORM FROM MAP TO WAYPOINT
                transform = self.tf_buffer.lookup_transform('odom', 'waypoint', rclpy.time.Time())

                # POSE DE WAYPOINT EN EL MUNDO
                self.poseWay.position.x = transform.transform.translation.x
                self.poseWay.position.y = transform.transform.translation.y
                self.poseWay.position.z = transform.transform.translation.z

                self.poseWay.orientation.x = transform.transform.rotation.x
                self.poseWay.orientation.y = transform.transform.rotation.y
                self.poseWay.orientation.z = transform.transform.rotation.z
                self.poseWay.orientation.w = transform.transform.rotation.w
                
        except Exception as e:
            self.get_logger().warn(f"Transform failed: {e}")
        
        # PUBLICAR POSE  DE WAYPONT
        self.pos_pub.publish(self.poseWay)


        # ODOM -> FOOTPRINT
        # -------------------------------------------------------------------------------------------
        self.t.header.stamp = self.get_clock().now().to_msg()
        self.t.header.frame_id = 'odom'
        self.t.child_frame_id = 'way_gay'

        self.t.transform.translation.x = self.poseWay.position.x
        self.t.transform.translation.y = self.poseWay.position.y 
        self.t.transform.translation.z = self.poseWay.position.z   

        self.t.transform.rotation.x = self.poseWay.orientation.x 
        self.t.transform.rotation.y = self.poseWay.orientation.y
        self.t.transform.rotation.z = self.poseWay.orientation.z
        self.t.transform.rotation.w = self.poseWay.orientation.w

        self.br1.sendTransform(self.t)
        # -------------------------------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)

    node = FramePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # Ensure shutdown is only called once
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()