import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(Image, 'webcam/image', 10)
        self.timer = self.create_timer(1.0 / 30, self.timer_callback)
        self.cap = cv2.VideoCapture(4) # webcam index
        self.bridge = CvBridge()
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open webcam")
            raise Exception("Webcam not found")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(image_msg)
            self.get_logger().info("Published image")
        else:
            self.get_logger().warning("Failed to capture image")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main():
    rclpy.init()
    node = WebcamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()