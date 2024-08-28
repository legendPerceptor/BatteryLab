import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from battery_lab_custom_msg.srv import CaptureImage
from cv_bridge import CvBridge
import cv2

class ImageClient(Node):
    def __init__(self):
        super().__init__('image_client')
        self.declare_parameter('service_name', '/batterylab/capture_image')
        service_name = self.get_parameter('service_name').get_parameter_value().string_value
        self.cli = self.create_client(CaptureImage, service_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = CaptureImage.Request()
        self.br = CvBridge()
        self.get_logger().info("The camera client is ready to get images...")

    def send_request(self):
        self.future = self.cli.call_async(self.req)

    def display_image(self):
        if self.future.done():
            try:
                response = self.future.result()
                current_frame = self.br.imgmsg_to_cv2(response.image, 'bgr8')
                cv2.imshow("camera", current_frame)
                cv2.waitKey(0)
            except Exception as e:
                self.get_logger().error(f'Service call failed {str(e)}')
        self.get_logger().info("finished executing display image")

def main(args=None):
    rclpy.init(args=args)
    image_client = ImageClient()
    image_client.send_request()

    rclpy.spin_until_future_complete(image_client, image_client.future)
    image_client.display_image()

    image_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
