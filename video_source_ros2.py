import threading
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, Duration
from sensor_msgs.msg import CompressedImage, Image

class ROS2VideoSource(Node):
    def __init__(self, topic):
        super().__init__('video_subscriber')
        self.bridge = CvBridge()
        self.frame = None
        self.frame_lock = threading.Lock()
        self.running = True
        self.compressed = True  # Set to True for CompressedImage, False for Image

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1            
        )

        # Subscribe to the ROS2 topic
        if self.compressed:
            self.subscription = self.create_subscription(            
                CompressedImage, topic, self.image_callback, qos_profile)
        else:
            self.subscription = self.create_subscription(
                Image, topic, self.image_callback, qos_profile)

        # Start spinning thread
        self.spin_thread = threading.Thread(target=self.spin, daemon=True)
        self.spin_thread.start()

    def image_callback(self, msg):
        """Callback to convert ROS Image message to OpenCV format"""
        try:
            if self.compressed:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.frame_lock:
                self.frame = cv_image
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def spin(self):
        """Spin the node to process messages"""
        while self.running:
            rclpy.spin_once(self, timeout_sec=0.1)

    def get_frame(self):
        """Return a copy of the latest frame"""
        with self.frame_lock:
            return self.frame.copy() if self.frame is not None else None

    def release(self):
        """Clean up resources"""
        self.running = False
        self.spin_thread.join()
        self.destroy_node()