import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import cv2
from pyzbar import pyzbar
from cv_bridge import CvBridge

class QRDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_detector_node')
        self.publisher_ = self.create_publisher(String, 'qr_code_data', 10)

        self.bridge = CvBridge()

        # self.timer = self.create_timer(0.1, self.detect_qr)
        # self.cap = cv2.VideoCapture(0)

        self.cam_sub_ = self.create_subscription(
            CompressedImage,
            '/signal_frame',
            self.cam_callback,
            10)
        self.cam_sub_  # prevent unused variable warning

        self.get_logger().info("Nodo QRDetectorNode inicializado.")

    # def detect_qr(self):
        # ret, frame = self.cap.read()
        # if not ret:
        #     self.get_logger().warn("No se pudo leer desde la cámara.")
        #     return

    # detect_qr -> cam_callback when listening to a topic

    def cam_callback(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg)

        decoded_objects = pyzbar.decode(frame)
        for obj in decoded_objects:
            data = obj.data.decode('utf-8')
            self.get_logger().info(f"QR detectado: {data}")
            msg = String()
            msg.data = data
            if msg.data == "circular":
                msg.data = "oxidado"
            self.publisher_.publish(msg)

            # Dibuja en la ventana
            (x, y, w, h) = obj.rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, data, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("QR Detector (ROS 2)", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = QRDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
