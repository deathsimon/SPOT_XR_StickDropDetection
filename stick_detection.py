import cv2
import configparser
import time
import threading
import numpy as np
from skimage.metrics import structural_similarity as ssim
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class StickDetector:    
    def __init__(self, config_file="config"):
        # Load configuration
        config = configparser.ConfigParser()
        config.read(config_file)

        if 'Settings' not in config:
            raise Exception(f"Error: 'Settings' section not found in {config_file}")
        
        # Read parameters from config
        self.video_source = config['Settings'].get('video_source', '')
        if self.video_source == 'RTSP':
            self.rtsp_url = config['Settings'].get('rtsp_url', '')  # RTSP URL for the stream
        elif self.video_source == 'Camera':
            self.camera_index = int(config['Settings'].get('camera_index', 0)) # Camera index if needed
        elif self.video_source == 'ROS2':
            self.video_topic = config['Settings'].get('video_topic', '')  # ROS2 topic for the stream
        else:
            raise Exception(f"Error: Invalid video source '{self.video_source}' in {config_file}")
                    
        self.verbose = config.getboolean('Settings', 'verbose', fallback=False)  # Verbose output
        self.target_IP = config['Settings'].get('IP', '') # IP address for signal
        self.target_port = config['Settings'].get('port', '') # IP address for signal
        self.fall_detection = int(config['Settings'].get('fall_detection_method', 0)) # Fall detection method
        if self.fall_detection == 0:
            self.drop_threshold = int(config['Settings'].get('MSE_threshold', 1000))    # Pixels
        else:
            self.drop_threshold = float(config['Settings'].get('SSIM_change_threshold', 0.5))    # SSIM
        
        # Initialize shared variables
        self.frame = None  # Latest frame from video stream
        self.frame_height = 0  # Frame dimensions
        self.frame_width = 0
        self.running = False
        self.frame_lock = threading.Lock()  # Lock for thread-safe frame access
        self.detecting = False
        self.aoi = None  # Area of interest: (x, y, w, h)
        self.prev_aoi = None  # Previous AOI image for comparison
        self.selecting_aoi = False  # Flag for AOI selection
        self.start_point = None  # For mouse drawing
        self.end_point = None  # For mouse drawing
        
        # Initialize video capture
        if self.video_source == 'RTSP':        
            self.cap = cv2.VideoCapture(self.rtsp_url)
            print(f"Using RTSP URL: {self.rtsp_url}")            
        elif self.video_source == 'Camera':
            self.cap = cv2.VideoCapture(self.camera_index)
            print(f"Using Camera Index: {self.camera_index}")
        elif self.video_source == 'ROS2':
            rclpy.init()
            self.bridge = CvBridge()
            self.cap = None        
                        
        if self.video_source != 'ROS2' and not self.cap.isOpened():
            raise Exception(f"Error: Could not open video stream")        
        
        # Set up link to Spot Arm via ROS2 client using target IP and port        
        from Spot_arm import SpotArm
        try:
            self.spot_arm = SpotArm(self.target_IP, self.target_port)
        except ConnectionError as e:
            print(f"Warning: {e}")
            self.spot_arm = None

    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events for AOI selection"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.detecting = False  # Stop detection while selecting AOI
            self.selecting_aoi = True
            # Clamp coordinates to frame boundaries
            x = max(0, min(x, self.frame_width - 1))
            y = max(0, min(y, self.frame_height - 1))
            self.start_point = (x, y)
            self.end_point = (x, y)            
        elif event == cv2.EVENT_MOUSEMOVE and self.selecting_aoi:
            # Clamp coordinates to frame boundaries
            x = max(0, min(x, self.frame_width - 1))
            y = max(0, min(y, self.frame_height - 1))
            self.end_point = (x, y)            
        elif event == cv2.EVENT_LBUTTONUP:
            self.selecting_aoi = False
            x1, y1 = self.start_point
            x2, y2 = self.end_point
            # Ensure valid rectangle
            x = min(x1, x2)
            y = min(y1, y2)
            w = abs(x2 - x1)
            h = abs(y2 - y1)
            # Clamp to frame boundaries
            x = max(0, min(x, self.frame_width - w))
            y = max(0, min(y, self.frame_height - h))
            w = min(w, self.frame_width - x)
            h = min(h, self.frame_height - y)
            if w > 0 and h > 0:
                self.aoi = (x, y, w, h)
                print(f"AOI selected: x={x}, y={y}, w={w}, h={h}")
                self.detecting = True
                self.prev_aoi = None  # Reset previous AOI for new selection
                print("Starting Stick Detection...")

    def draw_status_tag(self, frame, x, y, status_tag="", bg_color=(0,0,0)):
        """Draw status tag above the AOI or at top-left corner"""
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        font_color = (255, 255, 255)  # White text
        thickness = 1

        # Get text size
        (text_w, text_h), baseline = cv2.getTextSize(status_tag, font, font_scale, thickness)
        
        # Position: Above AOI if set, else top-left corner
        text_x = x
        text_y = max(y - text_h - 5, 5)  # 5 pixels above AOI, avoid top edge

        # Draw background rectangle
        cv2.rectangle(frame, (text_x, text_y - text_h), 
                     (text_x + text_w, text_y + baseline), bg_color, -1)
        # Draw text
        cv2.putText(frame, status_tag, (text_x, text_y), font, font_scale, 
                    font_color, thickness, cv2.LINE_AA)                
        
    
    def check_connection(self):
        if self.spot_arm is None:
            print("Warning: Action not performed. Spot Arm not initialized.")
            return False
        return True               

    def compute_ssim(self, img1, img2):
        """Compute SSIM between two images"""         
        if img1.shape != img2.shape:
            return -1.0
        try:
            score = ssim(img1, img2, channel_axis=-1, data_range=255)
            return score
        except ValueError:            
            return -1.0  # Handle errors (e.g., empty images)    

    def compute_mse(self, img1, img2):
        """Compute mean squared error between two images"""
        if img1.shape != img2.shape:
            return float('inf')  # Invalid comparison
        err = np.sum((img1.astype("float") - img2.astype("float")) ** 2)
        err /= float(img1.shape[0] * img1.shape[1])
        return err
        
    def check_drop(self, frame):        
        """Check for drastic pixel changes in AOI"""
        if self.aoi is None or self.prev_aoi is None:
            return
        
        x, y, w, h = self.aoi
        # Extract current AOI
        current_aoi = frame[y:y+h, x:x+w]

        if current_aoi.size == 0 or current_aoi.shape != self.prev_aoi.shape:
            return

        if self.fall_detection == 0:
            # Compute MSE
            changes = self.compute_mse(current_aoi, self.prev_aoi)
            if self.verbose:
                print(f"Current AOI MSE: {changes:.2f}")
        else:
            # Compute SSIM
            changes = self.compute_ssim(current_aoi, self.prev_aoi)            
            if changes < 0:
                print("Error: SSIM computation failed.")
                return
            else:
                # Invert SSIM to get a change metric
                changes = 1 - changes
            if self.verbose:
                print(f"Current AOI SSIM: {changes:.2f}")            
        
        # Check if change exceeds threshold
        if changes > self.drop_threshold:
            print("[" + str(time.time_ns()) +"] Signal: Stick has dropped!")
            if self.spot_arm is not None:
                self.spot_arm.close_gripper()  # Close gripper
                print("Spot: Arm gripper closed.")
                print("Did Spot catch the stick?")
            # self.send_ws_signal(0x2d) # Send signal
            self.detecting = False  # Stop detection after drop

        # Update previous AOI
        # self.prev_aoi = current_aoi.copy()    

    def capture_stream(self):
        """Thread to continuously capture frames from video stream"""
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                print("Error: Could not read frame.")
                self.running = False
                break

            # Update shared frame with thread-safe access
            with self.frame_lock:
                self.frame = frame
                self.frame_height, self.frame_width = frame.shape[:2]

            # Small sleep to prevent excessive CPU usage (adjust as needed)
            time.sleep(0.001)  # 1 ms

    def image_callback(self, msg):
        """Callback to handle incoming ROS 2 image messages"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.frame_lock:
                self.frame = cv_image
                self.frame_height, self.frame_width = cv_image.shape[:2]
        except Exception as e:
            print(f"Error converting ROS Image to OpenCV: {e}")

    def subscribe_ros_topic(self):
        """Thread to subscribe to ROS 2 image topic"""
        node = rclpy.create_node('image_subscriber')
        subscription = node.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10)  # QoS profile depth of 10
        rclpy.spin(node)  # Spin the node to process messages
        node.destroy_node()  # Cleanup when spinning stops

    def run(self):
        """Main thread: Perform pixel change detection and display"""        
        self.running = True
        # self.detecting = True
        # print("Starting Stick Detection...")
        
        if self.video_source == 'ROS2':
            ros_thread = threading.Thread(target=self.subscribe_ros_topic, daemon=True)
            ros_thread.start()
        else:
            # Start RTSP/Camera capture thread
            video_thread = threading.Thread(target=self.capture_stream, daemon=True)
            video_thread.start()

        # Set up window and mouse callback for AOI selection        
        cv2.namedWindow("Stick Detection", flags=cv2.WINDOW_GUI_NORMAL)
        cv2.setMouseCallback("Stick Detection", self.mouse_callback)

        # Main loop: Detection and display
        try:
            while self.running:
                # Get the latest frame
                with self.frame_lock:
                    if self.frame is None:
                        continue  # Skip if no frame yet
                    frame = self.frame.copy()  # Work on a copy

                
                # Draw AOI rectangle if selecting or selected
                if self.selecting_aoi and self.start_point and self.end_point:
                    x1, y1 = self.start_point
                    x2, y2 = self.end_point
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    self.draw_status_tag(frame, x1, y1, "Selecting", (255, 0, 0))
                elif self.aoi:
                    x, y, w, h = self.aoi                    
                    if self.detecting:
                        aoi_color = (0, 255, 0)  # Green for detecting
                        status_tag = "Detecting"
                    else:
                        aoi_color = (0, 0, 255)  # Red for not detecting
                        status_tag = "Suspend"
                    cv2.rectangle(frame, (x, y), (x+w, y+h), aoi_color, 2)
                    self.draw_status_tag(frame, x, y, status_tag, aoi_color)
                    
                # Initialize prev_aoi with first valid AOI
                if self.aoi and self.prev_aoi is None:
                    x, y, w, h = self.aoi
                    self.prev_aoi = frame[y:y+h, x:x+w].copy()

                # Perform detection
                if self.detecting:
                    # Check for drop
                    start_time = time.time()
                    self.check_drop(frame)
                    end_time = time.time()
                    if self.verbose:
                        print(f"Drop Check Time: {end_time - start_time:.3f} seconds")
                
                # Display the frame                
                cv2.imshow("Stick Detection", frame)

                
                key = cv2.waitKey(5) & 0xFF
                if key == ord('q'):
                    # Check for quit key
                    self.running = False                    
                    video_thread.join()  # Wait for thread to finish
                    break                
                elif key == ord('d'):
                    # Restart detection after pressing 'd'
                    self.prev_aoi = None # Reset previous AOI
                    self.detecting = True
                    print("Detection restarted.")
                elif key == ord('r'):
                    # Reset Spot Arm                    
                    if self.check_connection() is True:
                        self.spot_arm.stand()
                        self.spot_arm.open_gripper_at_angle(27)
                        self.spot_arm.set_arm_joints(0.0, -1.2, 1.9, 0.0, -0.7, 1.57)
                        # self.spot_arm.set_arm_velocity(0.0, 0.2, 0.0) # nudge left
                        # self.spot_arm.set_arm_velocity(0.0, -0.2, 0.0) # nudge right
                        print("Spot: Arm to default position.")                    
                elif key == ord('o'):
                    if self.check_connection() is True:
                        self.spot_arm.open_gripper()
                        print("Spot: Arm gripper opened.")
                elif key == ord('s'):
                    if self.check_connection() is True:
                        self.spot_arm.close_gripper()
                        self.spot_arm.arm_stow()
                        self.spot_arm.sit()
                        print("Spot: Arm stowed and sitting.")
                elif key == ord('c'):
                    if self.check_connection() is True:
                        self.spot_arm.close_gripper()
                        print("Spot: Arm gripper closed.")


        except KeyboardInterrupt:
            print("Stopped by user.")
            self.running = False

        # Cleanup
        finally:
            self.cap.release()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    # Initialize detector with config file
    detector = StickDetector(config_file="config")
    detector.run()