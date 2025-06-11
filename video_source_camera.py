import threading
import cv2

class CameraVideoSource:
    def __init__(self, camera_index=0):
        """Initialize the video source from a camera."""             
        self.capture = cv2.VideoCapture(camera_index)
        fps = self.capture.get(cv2.CAP_PROP_FPS)
        print(f"Capturing video stream {camera_index} at {fps} frames per second.")

        
        if not self.capture.isOpened():
            raise RuntimeError(f"Could not open camera at index {camera_index}")

        self.frame = None
        self.frame_lock = threading.Lock()
        self.frame_available = threading.Semaphore()
        self.running = True

        # Start the video capture thread
        self.capture_thread = threading.Thread(target=self.capture_frames, daemon=True)
        self.capture_thread.start()

    def capture_frames(self):
        """Continuously capture frames from the camera."""
        while self.running:
            ret, frame = self.capture.read()
            if ret:
                self.frame_available.release()
                with self.frame_lock:
                    self.frame = frame
            else:
                print("Failed to capture frame from camera.")

    def get_frame(self):
        """Returns the latest frame (only unique, blocking)."""
        self.frame_available.acquire()
        with self.frame_lock:
            return self.frame if self.frame is not None else None
        
    def release(self):
        """Clean up resources."""
        self.running = False
        self.capture_thread.join()
        if self.capture.isOpened():
            self.capture.release()