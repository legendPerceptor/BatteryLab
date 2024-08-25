import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from battery_lab_custom_msg.srv import CaptureImage

class CameraService(Node):
    def __init__(self):
        super().__init__('camera_service')
        self.srv = self.create_service(CaptureImage, '/batterylab/capture_image', self.capture_image_callback)
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()
        self.get_logger().info("The camera server is ready accept requests...")

    def capture_image_callback(self, request, response):
        ret, frame = self.cap.read()
        if ret:
            response.image = self.br.cv2_to_imgmsg(frame, 'bgr8')
        else:
            self.get_logger().error('Failed to capture image')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CameraService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()