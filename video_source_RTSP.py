import cv2
import threading

class RTSPVideoSource:
    def __init__(self, rtsp_url=''):
        self.capture = cv2.VideoCapture(rtsp_url)
        
        if not self.capture.isOpened():
            raise RuntimeError(f"Could not open RTSP stream at {rtsp_url}")

        self.frame = None
        self.frame_lock = threading.Lock()
        self.running = True

        # Start the video capture thread
        self.capture_thread = threading.Thread(target=self.capture_frames, daemon=True)
        self.capture_thread.start()

    def capture_frames(self):
        """Continuously capture frames from the RTSP stream."""
        while self.running:
            ret, frame = self.capture.read()
            if ret:
                with self.frame_lock:
                    self.frame = frame
            else:
                print("Failed to capture frame from RTSP stream.")

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