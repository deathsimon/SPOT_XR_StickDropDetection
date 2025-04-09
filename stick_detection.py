import cv2
from ultralytics import YOLO
import configparser
import time
import threading

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
        self.drop_threshold = int(config['Settings'].get('drop_threshold', 100))    # Pixels
        self.target_class_id = int(config['Settings'].get('target_class_id', 39))   # Class ID to detect
        self.yolo_model = config['Settings'].get('yolo_model', 'yolo11n.pt')   # Path to YOLO model
        self.verbose = config.getboolean('Settings', 'verbose', fallback=False)  # Verbose output
        
        # Initialize shared variables
        self.frame = None  # Latest frame from RTSP
        self.prev_y = None
        self.current_y = None
        self.stick_detected = False        
        self.running = False
        self.frame_lock = threading.Lock()  # Lock for thread-safe frame access
        self.detecting = False
        
        # Load YOLO model
        self.model = YOLO(self.yolo_model)

        # Initialize video capture
        if self.rtsp_url:
            self.cap = cv2.VideoCapture(self.rtsp_url)
        else:
            self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise Exception(f"Error: Could not open video stream")
        
    def check_drop(self):        
        """Check if the stick has dropped based on Y-coordinate"""
        if self.stick_detected and self.prev_y is not None: # Check if stick was detected
            if self.current_y - self.prev_y > self.drop_threshold:
                print("Signal: Stick has dropped!")
                # Send signal here (e.g., HTTP request, GPIO trigger)
                self.detecting = False  # Stop detection after drop

    def detect_stick(self, frame):
        """Run object detection with YOLO and return frame with bounding box"""
        if not self.detecting:
            return frame # Skip detection if not detecting
            
        start_time = time.time()        
        # Run YOLO inference with verbose=False to silence output
        results = self.model(frame, verbose=self.verbose)
        end_time = time.time()
        if not self.verbose:
            print(f"YOLO Inference Time: {end_time - start_time:.3f} seconds")

        stick_detected_this_frame = False
        self.current_y = None

        for result in results:
            boxes = result.boxes
            for box, cls in zip(boxes.xywh, boxes.cls):
                if int(cls) == self.target_class_id:  # Match target class                    
                    x, y, w, h = box
                    stick_detected_this_frame = True
                    self.current_y = int(y)
                    if self.verbose:
                        print(f"Detected stick at Y: {self.current_y}")
                    # Draw bounding box                    
                    cv2.rectangle(frame, (int(x - w/2), int(y - h/2)), 
                                  (int(x + w/2), int(y + h/2)), (0, 255, 0), 2)
                    break  # Stop after finding the target class
            if stick_detected_this_frame:
                break  # Exit outer loop if stick is found
        
        # If stick is detected, check for drop and update previous position
        if stick_detected_this_frame:
            self.stick_detected = True
            self.check_drop()                
            self.prev_y = self.current_y
            
        return frame

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
            time.sleep(0.001)  #    

    def run(self):
        """Main thread: Perform detection, drop check, and display"""        
        self.running = True
        self.detecting = True        
        
        # Start RTSP capture thread
        video_thread = threading.Thread(target=self.capture_stream, daemon=True)
        video_thread.start()

        try:
            while self.running:
                # Get the latest frame
                with self.frame_lock:
                    if self.frame is None:
                        continue  # Skip if no frame yet
                    frame = self.frame.copy()  # Work on a copy

                # Perform detection and draw bounding box
                frame = self.detect_stick(frame)
                
                # Display the frame
                cv2.imshow("Stick Detection", frame)

                # Check for quit key
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.running = False
                    break

                # Restart detection after pressing 'd'
                if cv2.waitKey(1) & 0xFF == ord('d'):                    
                    self.prev_y = None
                    self.stick_detected = False  
                    self.detecting = True

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