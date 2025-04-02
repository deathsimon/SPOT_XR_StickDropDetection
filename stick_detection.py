import cv2
from ultralytics import YOLO

class StickDetector:
    def __init__(self, camera_index=0, target_class_id=39, drop_threshold=50):
        # Initialize variables
        self.prev_y = None
        self.current_y = None
        self.stick_detected = False
        self.drop_threshold = drop_threshold  # Pixels
        self.target_class_id = target_class_id  # Class ID to detect (e.g., 39 for "bottle")
        self.frame = None  # Store the latest frame        
        
        # Load YOLO model
        self.model = YOLO("yolo11n.pt")  # Nano version for speed

        # Initialize video capture
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise Exception(f"Error: Could not open video stream at index {camera_index}")
        
    def check_drop(self):        
        """Check if the stick has dropped based on Y-coordinate"""
        if self.stick_detected and self.prev_y is not None: # Check if stick was detected
            if self.current_y - self.prev_y > self.drop_threshold:
                print("Signal: Stick has dropped!")
                # Send signal here (e.g., HTTP request, GPIO trigger)        

    def detect_stick(self, frame):
        """Detect the stick in the frame and draw bounding box"""
        # Run YOLO inference with verbose=False to silence output
        results = self.model(frame, verbose=False)
        stick_detected_this_frame = False
        self.current_y = None

        for result in results:
            boxes = result.boxes
            for box, cls in zip(boxes.xywh, boxes.cls):
                if int(cls) == self.target_class_id:  # Match target class                    
                    x, y, w, h = box
                    stick_detected_this_frame = True
                    self.current_y = int(y)
                    print(f"Detected stick at Y: {self.current_y}")                    
                    # Draw bounding box                    
                    cv2.rectangle(frame, (int(x - w/2), int(y - h/2)), 
                                 (int(x + w/2), int(y + h/2)), (0, 255, 0), 2)
                    break  # Stop after finding the target class
            if stick_detected_this_frame:
                break  # Exit outer loop if stick is found
        
        # If stick is detected, check for drop and update previous position
        if stick_detected_this_frame:
            self.check_drop()
            self.stick_detected = True
            self.prev_y = self.current_y
                
        return frame

    def run(self):
        """Main loop to process video stream"""
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("Error: Could not read frame.")
                    break

                # Detect stick and draw bounding box
                frame = self.detect_stick(frame)

                # Display the frame
                cv2.imshow("Stick Detection", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        except KeyboardInterrupt:
            print("Stopped by user.")

        # Cleanup
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Initialize detector with camera index and target class ID
    detector = StickDetector(camera_index=4, target_class_id=39)  # 39 = "bottle" in COCO, change as needed (e.g., 36 = "baseball bat")
    detector.run()