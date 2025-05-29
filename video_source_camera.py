import threading
import cv2

class CameraVideoSource:
    def __init__(self, camera_index=0):
        # super().__init__('video_source_node')        
        self.capture = cv2.VideoCapture(camera_index)
        
        if not self.capture.isOpened():
            raise RuntimeError(f"Could not open camera at index {camera_index}")

        self.frame = None
        self.frame_lock = threading.Lock()
        self.running = True

        # Start the video capture thread
        self.capture_thread = threading.Thread(target=self.capture_frames, daemon=True)
        self.capture_thread.start()

    def capture_frames(self):
        """Continuously capture frames from the camera."""
        while self.running:
            ret, frame = self.capture.read()
            if ret:
                with self.frame_lock:
                    self.frame = frame
            else:
                self.get_logger().error("Failed to capture frame from camera.")

    def get_frame(self):
        """Return a copy of the latest frame."""
        with self.frame_lock:
            return self.frame.copy() if self.frame is not None else None
        
    def release(self):
        """Clean up resources."""
        self.running = False
        self.capture_thread.join()
        if self.capture.isOpened():
            self.capture.release()        