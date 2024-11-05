import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraSubscriber(Node):
    def __init__(self, camera_topic=None):
        super().__init__("camera_subscriber")
        self.declare_parameter("camera_topic", "/camera/tower_camera")
        topic = (
            camera_topic
            if camera_topic is not None
            else self.get_parameter("camera_topic").get_parameter_value().string_value
        )
        self.subscription = self.create_subscription(
            Image, topic, self.listener_callback, 10
        )
        self.br = CvBridge()

    def listener_callback(self, msg):
        cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow("Camera Feed", cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    try:
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        print("camera client interrupted by user.")
    finally:
        camera_subscriber.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
