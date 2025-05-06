import cv2
import configparser
import time
import threading
import numpy as np
from skimage.metrics import structural_similarity as ssim
from websockets.sync.client import connect


class StickDetector:    
    def __init__(self, config_file="config"):
        # Load configuration
        config = configparser.ConfigParser()
        config.read(config_file)

        if 'Settings' not in config:
            raise Exception(f"Error: 'Settings' section not found in {config_file}")
        
        # Read parameters from config
        self.rtsp_url = config['Settings'].get('rtsp_url', '')  # RTSP URL for the stream
        self.camera_index = int(config['Settings'].get('camera_index', 0)) # Camera index if needed        
        self.verbose = config.getboolean('Settings', 'verbose', fallback=False)  # Verbose output
        self.target_IP = config['Settings'].get('IP', '') # IP address for signal
        self.target_port = config['Settings'].get('port', '') # IP address for signal
        self.fall_detection = int(config['Settings'].get('fall_detection_method', 0)) # Fall detection method
        if self.fall_detection == 0:
            self.drop_threshold = int(config['Settings'].get('MSE_threshold', 1000))    # Pixels
        else:
            self.drop_threshold = float(config['Settings'].get('SSIM_change_threshold', 0.5))    # SSIM
        
        # Initialize shared variables
        self.frame = None  # Latest frame from RTSP
        self.running = False
        self.frame_lock = threading.Lock()  # Lock for thread-safe frame access
        self.detecting = False
        self.aoi = None  # Area of interest: (x, y, w, h)
        self.prev_aoi = None  # Previous AOI image for comparison
        self.selecting_aoi = False  # Flag for AOI selection
        self.start_point = None  # For mouse drawing
        self.end_point = None  # For mouse drawing
        
        # Initialize video capture
        if self.rtsp_url != '':
            self.cap = cv2.VideoCapture(self.rtsp_url)
            print(f"Using RTSP URL: {self.rtsp_url}")
        else:
            self.cap = cv2.VideoCapture(self.camera_index)
            print(f"Using Camera Index: {self.camera_index}")
        if not self.cap.isOpened():
            raise Exception(f"Error: Could not open video stream")                

    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events for AOI selection"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.detecting = False  # Stop detection while selecting AOI
            self.selecting_aoi = True
            self.start_point = (x, y)
            self.end_point = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.selecting_aoi:
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

    def send_ws_signal(self, signal):
        if self.target_IP == '' or self.target_port == '':
            print("No target IP or port specified. Signal not sent.")
            return
        
        with connect(f"ws://{self.target_IP}:{self.target_port}") as client:
            client.send(signal.to_bytes(1, byteorder="big"))

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
            print("Signal: Stick has dropped!")
            self.send_ws_signal(0x2d) # Send signal
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

            # Small sleep to prevent excessive CPU usage (adjust as needed)
            time.sleep(0.001)  # 1 ms

    def run(self):
        """Main thread: Perform pixel change detection and display"""        
        self.running = True
        # self.detecting = True
        # print("Starting Stick Detection...")
        
        # Start RTSP capture thread
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
                    break                
                elif key == ord('d'):
                    # Restart detection after pressing 'd'
                    self.prev_aoi = None # Reset previous AOI
                    self.detecting = True
                    print("Detection restarted.")  

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