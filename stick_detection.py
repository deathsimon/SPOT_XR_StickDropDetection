import cv2
import configparser
import time

# import threading
import numpy as np
import serial
from rodholderclient import HolderClient

from collections import deque
from skimage.metrics import structural_similarity as ssim
from PySide6.QtGui import QImage, QMouseEvent, QKeyEvent
from PySide6.QtCore import QThread, Signal, QEvent, QObject, Qt
from PySide6.QtWidgets import QLabel, QStyle, QWidget

class StickDetector(QThread):
    updateFrame = Signal(QImage)
    
    def _update_labels(self):
        # handle labels to display status
        window: QWidget = self.parent().window
        # if self.detecting:
        #     window.fallDetectionStatus.setText("ACTIVE")
        #     window.fallDetectionStatus.setStyleSheet("background-color: #90BE6D;")
        # else:
        #     window.fallDetectionStatus.setText("INACTIVE")
        #     window.fallDetectionStatus.setStyleSheet("background-color: #ff0000;")
        
        # if self.stick_state == "Raised":
        #     window.rodStatus.setText("RAISED")
        #     window.rodStatus.setStyleSheet("background-color: #90BE6D;")
        # elif self.stick_state == "Dropped":
        #     window.rodStatus.setText("DROPPED")
        #     window.rodStatus.setStyleSheet("background-color: #ff0000;")
        # else:
        #     pass
        
        # if self.spot_arm:
        #     window.spotStatus.setText("CONNECTED")
        #     window.spotStatus.setStyleSheet("background-color: #90BE6D;")
        # else:
        #     window.spotStatus.setText("DISCONNECTED")
        #     window.spotStatus.setStyleSheet("background-color: #ff0000;")

    
    def _reload_settings(self, path: str):
        self.reloadable_config = path
        
        parser = configparser.ConfigParser()
        parser.read(path)
        
        aoi_str = parser['RuntimeSettings'].get("aoi", "None")
        if aoi_str != "None":
            self.aoi = tuple(int(x) for x in aoi_str.removeprefix('(').removesuffix(')').split(','))
            print(f"Set AOI from config to: {self.aoi}")
            self.detecting = False
        else:        
            self.aoi = None  # Area of interest: (x, y, w, h)
            self.detecting = False

        self.fall_detection = parser['RuntimeSettings'].getint("fall_detection_method", 0) # Fall detection method
        if self.fall_detection == 0:
            self.drop_threshold = parser['RuntimeSettings'].getint("MSE_threshold", 100)   # Pixels
        else:
            self.drop_threshold = parser['RuntimeSettings'].getfloat("SSIM_change_threshold", 0.2)   # SSIM
        
        self.angle_tol = parser['RuntimeSettings'].getfloat("of_angle_tolerance", 0.2)   # optical flow
        self.magnitude_thresh = parser['RuntimeSettings'].getfloat("of_magnitude_thres", 2.5)   # optical flow
        
        self.stored_frames = parser['RuntimeSettings'].getint("stored_frames", 1)
        self.required_frames = parser['RuntimeSettings'].getint("required_frames", 1)
        assert self.stored_frames >= self.required_frames, "You cannot require more than you store!"
        
        self.frame_thresholds = deque([], maxlen=self.stored_frames)
        
        # we need to update the labels (if we can)
        try:
            self._update_labels()
        except:
            pass

    def _save_aoi(self):
        parser = configparser.ConfigParser()
        parser.read(self.reloadable_config)
        parser.set("RuntimeSettings", "aoi", str(self.aoi))
        parser.write(open(self.reloadable_config, "w"))
    
    def __init__(self, parent, config_file="config", reloadable_config="hot_load_config"):
        # handle Qt
        QThread.__init__(self, parent)

        # Load configuration
        config = configparser.ConfigParser()
        config.read(config_file)

        if "Settings" not in config:
            raise Exception(f"Error: 'Settings' section not found in {config_file}")

        # Read parameters from config
        self.video_source = config["Settings"].get("video_source", "")
        if self.video_source not in ["RTSP", "Camera", "ROS2"]:
            raise Exception(
                f"Error: Invalid video source '{self.video_source}' in {config_file}"
            )

        self.verbose = config.getboolean(
            "Settings", "verbose", fallback=False
        )  # Verbose output
        self.show_fps = config.getboolean(
            "Settings", "fps_counter", fallback=True
        )  # Show fps counter
        self.target_IP = config["Settings"].get("IP", "")  # IP address for signal
        self.target_port = config["Settings"].get("port", "")  # IP address for signal
        self.fall_detection = int(
            config["Settings"].get("fall_detection_method", 0)
        )  # Fall detection method
        if self.fall_detection == 0:
            self.drop_threshold = int(
                config["Settings"].get("MSE_threshold", 1000)
            )  # Pixels
        else:
            self.drop_threshold = float(
                config["Settings"].get("SSIM_change_threshold", 0.5)
            )  # SSIM

        self.stored_frames = int(config["Settings"].get("stored_frames", 3))
        self.required_frames = int(config["Settings"].get("required_frames", 2))
        assert self.stored_frames >= self.required_frames, (
            "You cannot require more than you store!"
        )

        if config["Settings"].getboolean("use_stick_ws", True):
            self.holder = HolderClient(
                host=config["Settings"].get("holderhost", "localhost"),
                port=config["Settings"].get("holderport", 8000)
            )
            self.stick_state = "Raised"
        else:
            self.stick_state = "???"
            print("Stick holder is disabled")

        # Initialize shared variables
        self.frame = None  # Latest frame from video stream
        self.frame_height = 0  # Frame dimensions
        self.frame_width = 0
        self.frame_thresholds = deque([], maxlen=self.stored_frames)
        self.running = False
        self.detecting = False
        self.prev_aoi = None  # Previous AOI image for comparison
        self.prev_aoi_2 = None  # Previous AOI image for optical flow comparison
        self.selecting_aoi = False  # Flag for AOI selection
        self.start_point = None  # For mouse drawing
        self.end_point = None  # For mouse drawing
        self.last_frame = None
        self.last_movement = deque(maxlen=1)
        self.total = 0
        self.stick_dropped = False
        self.stick_drop_command_time = None

        if config["Settings"].get("aoi", None):
            aoi_parts = config["Settings"].get("aoi").split(",")
            if len(aoi_parts) == 4:
                self.aoi = tuple([int(part) for part in aoi_parts])
                print(f"will be using AOI: {self.aoi}")
            else:
                print("Error: AOI could not be initialized! Please check format")

        # read the reloadable configs
        self._reload_settings(reloadable_config)
        self.stick_dropped = False
        self.stick_drop_command_time = None


        # Initialize video capture
        if self.video_source == "RTSP":
            rtsp_url = config["Settings"].get("rtsp_url", "")  # RTSP URL for the stream
            from video_source_RTSP import RTSPVideoSource

            self.video_stream = RTSPVideoSource(rtsp_url)
            print(f"Using RTSP URL: {rtsp_url}")
        elif self.video_source == "Camera":
            camera_index = int(
                config["Settings"].get("camera_index", 0)
            )  # Camera index if needed
            from video_source_camera import CameraVideoSource

            self.video_stream = CameraVideoSource(camera_index)
            print(f"Using Camera Index: {camera_index}")
        elif self.video_source == "ROS2":
            import rclpy

            rclpy.init()
            from video_source_ros2 import ROS2VideoSource

            video_topic = config["Settings"].get(
                "video_topic", ""
            )  # ROS2 topic for the stream
            self.video_stream = ROS2VideoSource(video_topic)

        # Set up link to Spot Arm via ROS2 client using target IP and port
        from Spot_arm import SpotArm

        try:
            self.spot_arm = SpotArm(self.target_IP, self.target_port)
        except ConnectionError as e:
            print(f"Warning: {e}")
            self.spot_arm = None
        
        # update labels of the dashboards
        self._update_labels()


    def eventFilter(self, obj : QObject, event : QEvent):
        event_map = {
            QEvent.Type.MouseButtonPress: cv2.EVENT_LBUTTONDOWN,
            QEvent.Type.MouseMove: cv2.EVENT_MOUSEMOVE,
            QEvent.Type.MouseButtonRelease: cv2.EVENT_LBUTTONUP,
        }
        ev_ty = event_map.get(event.type(), None)
        if ev_ty is not None:
            mv = QMouseEvent(event)
            if mv.button() == Qt.MouseButton.RightButton:
                self.aoi = None
                self.detecting = False
                # after some input we update our labels
                self._update_labels()
                return True

            # remap to actual frame size
            label: QLabel = obj.window.video_stream
            pxy = label.mapFrom(self.parent().window, mv.pos())
            x, y = pxy.x() - 8, pxy.y()
            
            # vertical center alignment, left align
            yoffset = (label.height() - label.pixmap().height()) / 2
            y = int(y - yoffset)
            
            # transform to opencv frame size
            w, h = obj.scaled_width, obj.scaled_height
            fw, fh = obj.frame_width, obj.frame_height
            x = int(x * (fw / w))
            y = int(y * (fh / h))

            # call the old callback
            self.mouse_callback(ev_ty, x, y, None, None)
            
            return True
        elif event.type() == QEvent.Type.KeyPress:
            kv = QKeyEvent(event)
            try:
                self.keyboard_callback(chr(kv.key()))
            except:
                pass
            return True
        else:
            return False

    def keyboard_callback(self, key: str):
        key = key.lower()
        if key == 'q':
            print("WARN: Close via window!")
        elif key == 'd':
            # Restart detection after pressing 'd'
            if self.aoi is None:
                print("Error: No AOI selected. Please select an AOI first.")
            self.prev_aoi = None # Reset previous AOI
            self.detecting = not self.detecting
            print("Detection toggled.")
        elif key == 'r':
            # Reset Spot Arm                    
            if self.check_connection() is True:
                self.spot_arm.stand()
                self.spot_arm.open_gripper_at_angle(35)
                self.spot_arm.set_arm_joints(0.0, -1.2, 1.9, 0.0, -0.7, 1.57)
                # self.spot_arm.set_arm_velocity(0.0, 0.2, 0.0) # nudge left
                # self.spot_arm.set_arm_velocity(0.0, -0.2, 0.0) # nudge right
                print("Spot: Arm to default position.")
        elif key == "o":
            if self.check_connection() is True:
                self.spot_arm.open_gripper()
                print("Spot: Arm gripper opened.")
        elif key == "s":
            if self.check_connection() is True:
                self.spot_arm.close_gripper()
                self.spot_arm.arm_stow()
                self.spot_arm.sit()
                print("Spot: Arm stowed and sitting.")
        elif key == "c":
            if self.check_connection() is True:
                self.spot_arm.close_gripper()
                print("Spot: Arm gripper closed.")
        elif key == "1":
            if self.holder and not self.stick_dropped:
                self.stick_dropped = True
                self.stick_drop_command_time = time.time()
                self.holder.drop()
                print("Dropped stick.")
        elif key == "2":
            if self.holder and self.stick_dropped:
                self.stick_drop_command_time = None
                self.holder.pullup()
                print("Initiated stick pull up.")
        elif key == 't':
            self._save_aoi()
            print("Saved current aoi.")
        elif key == 'z':
            self._reload_settings(self.reloadable_config)
            print("Reloaded current config.")
        
        # after some input we update our labels
        self._update_labels()


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
        # after some input we update our labels
        self._update_labels()

    def draw_status_tag(self, frame, x, y, status_tag="", bg_color=(0, 0, 0)):
        """Draw status tag above the AOI or at top-left corner"""
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        font_color = (255, 255, 255)  # White text
        thickness = 1

        # Get text size
        (text_w, text_h), baseline = cv2.getTextSize(
            status_tag, font, font_scale, thickness
        )

        # Position: Above AOI if set, else top-left corner
        text_x = x
        text_y = max(y - text_h - 5, 5)  # 5 pixels above AOI, avoid top edge

        # Draw background rectangle
        cv2.rectangle(
            frame,
            (text_x, text_y - text_h),
            (text_x + text_w, text_y + baseline),
            bg_color,
            -1,
        )
        # Draw text
        cv2.putText(
            frame,
            status_tag,
            (text_x, text_y),
            font,
            font_scale,
            font_color,
            thickness,
            cv2.LINE_AA,
        )

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
            return float("inf")  # Invalid comparison
        err = np.sum((img1.astype("float") - img2.astype("float")) ** 2)
        err /= float(img1.shape[0] * img1.shape[1])
        return err
    
    def compute_opflow(self, prev_aoi, curr_aoi) -> bool:
        """Compute downwards optical flow two images"""


        grey_prev_aoi = cv2.cvtColor(prev_aoi, cv2.COLOR_BGR2GRAY)
        grey_curr_aoi = cv2.cvtColor(curr_aoi, cv2.COLOR_BGR2GRAY)

        # calculate flow
        flow = cv2.calcOpticalFlowFarneback(
            grey_prev_aoi,
            grey_curr_aoi,
            None,
            pyr_scale=0.5,
            levels=1,
            winsize=6,
            iterations=2,
            poly_n=2,
            poly_sigma=1.2,
            flags=0,
        )

        # mag_img, pha_img = cv2.cartToPolar(
        #     flow[..., 0], flow[..., 1], angleInDegrees=True
        # )

        # filtered = np.where(
        #     ((pha_img < 110) & (pha_img > 60)) & (mag_img > 6.0), mag_img, 0
        # )
        # w,h = self.aoi[2:4]
        # area = w*h

        # normed_change = filtered.sum()/area
        # if normed_change > 0.1:
        #     print(f"{normed_change=}")
        # self.last_movement.append(normed_change)



        # if np.all(np.array(self.last_movement) > 10000):
        # if np.all(np.array(self.last_movement) > 0.1):
        #     self.total += 1
        #     if self.check_connection() is True:
        #         self.spot_arm.close_gripper()  # Close gripper
        #         print("Spot: Arm gripper closed.")
        #     print(f"Found something! {self.total}x")
        #     print("Stick has dropped!")
        #     # self.send_ws_signal(0x2d) # Send signal
        #     self.detecting = False  # Stop detection after drop
        #     return True

        # Calculate the magnitude and angle (in radians)
        magnitude, angle = cv2.cartToPolar(flow[..., 0], flow[..., 1], angleInDegrees=False)

        # Create a mask for angles close to downward direction (π/2)
        down_mask = (angle > (np.pi/2 - self.angle_tol)) & (angle < (np.pi/2 + self.angle_tol))

        # Optionally, threshold by magnitude to avoid noise
        significant_motion = magnitude > self.magnitude_thresh

        # print(f"{magnitude=}")

        # Combine both masks
        falling_pixels = down_mask & significant_motion


        # print(f"{falling_pixels=}")

        # Count or visualize
        if np.any(falling_pixels):
            print("Downward motion detected!")
            if self.check_connection() is True:
                self.spot_arm.close_gripper()  # Close gripper
                print("Spot: Arm gripper closed.")
            print(f"Found something! {self.total}x")
            print("Stick has dropped!")
            # self.send_ws_signal(0x2d) # Send signal
            self.detecting = False  # Stop detection after drop
            return True
        # else:
        #     print("No significant downward motion.")

        return False


    def check_drop(self, frame):
        """Check for drastic pixel changes in AOI"""
        if self.aoi is None or self.prev_aoi is None:
            return

        x, y, w, h = self.aoi
        # Extract current AOI
        current_aoi = frame[y : y + h, x : x + w]

        if current_aoi.size == 0 or current_aoi.shape != self.prev_aoi.shape:
            return

        if self.fall_detection == 0:
            # Compute MSE
            changes = self.compute_mse(current_aoi, self.prev_aoi)
            if self.verbose:
                print(f"Current AOI MSE: {changes:.2f}")
        elif self.fall_detection == 1:
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
        elif self.fall_detection == 2:
            changes = self.compute_opflow(self.prev_aoi, current_aoi)

        # Check if change exceeds threshold
        if self.fall_detection < 2:
            self.frame_thresholds.append(changes)
            if len([t for t in self.frame_thresholds if t > self.drop_threshold]) >= self.required_frames:
                print("Stick has dropped!")
                self.frame_thresholds.clear()
                if self.check_connection() is True:
                    self.spot_arm.close_gripper()  # Close gripper
                    print("Spot: Arm gripper closed.")
                    print("Did Spot catch the stick?")
                # self.send_ws_signal(0x2d) # Send signal
                self.detecting = False  # Stop detection after drop
        else :
            return changes



    def check_drop_cni(self, frame):
        """Check for drastic pixel changes in AOI"""
        if self.aoi is None and self.prev_aoi_2 is None:
            return
        
        current_aoi = None

        x, y, w, h = self.aoi
        # print("Actually comparing 1")
        if self.prev_aoi_2 is None:
            # x, y, w, h = self.aoi
            current_aoi = frame[y : y + h, x : x + w]
            self.prev_aoi_2 = cv2.cvtColor(current_aoi, cv2.COLOR_BGR2GRAY)
            return

        # print("Actually comparing 2")

        if current_aoi == None:
            # if x == None:
            #     x, y, w, h = self.aoi
            current_aoi = frame[y : y + h, x : x + w]
        grey_current_aoi = cv2.cvtColor(current_aoi, cv2.COLOR_BGR2GRAY)


        # print("Actually comparing 3")

        if current_aoi.size == 0:
            return


        # print("Actually comparing 4")

        # print(f"{grey_current_aoi.shape=}, {self.prev_aoi_2.shape=}")
        
        if grey_current_aoi.shape != self.prev_aoi_2.shape:
            self.last_frame = frame
            self.prev_aoi_2 = grey_current_aoi
            return


        # print("Actually comparing 5")

        # calculate flow
        flow = cv2.calcOpticalFlowFarneback(
            self.prev_aoi_2,
            grey_current_aoi,
            None,
            pyr_scale=0.5,
            levels=1,
            winsize=3,
            iterations=1,
            poly_n=2,
            poly_sigma=1.2,
            flags=0,
        )

        mag_img, pha_img = cv2.cartToPolar(
            flow[..., 0], flow[..., 1], angleInDegrees=True
        )

        filtered = np.where(
            ((pha_img < 110) & (pha_img > 60)) & (mag_img > 6.0), mag_img, 0
        )

        area = w*h
        normed_change = filtered.sum()/area
        if normed_change > 0.1:
            print(f"{normed_change=}")
        self.last_movement.append(normed_change)



        # if np.all(np.array(self.last_movement) > 10000):
        if np.all(np.array(self.last_movement) > 0.1):
            self.total += 1
            print(f"Found something! {self.total}x")
        self.frame_thresholds.append(changes)
        if len([t for t in self.frame_thresholds if t > self.drop_threshold]) >= self.required_frames:
            print("Stick has dropped!")
            self.frame_thresholds.clear()
            if self.check_connection() is True:
                self.spot_arm.close_gripper()  # Close gripper
                print("Spot: Arm gripper closed.")
                print("Did Spot catch the stick?")
            self.detecting = False  # Stop detection after drop
            
            # update our labels
            self._update_labels()

        # Update previous AOI
        # self.prev_aoi = current_aoi.copy()    

    def run(self):
        """Main thread: Perform pixel change detection and display"""
        self.running = True
        self.last_time = time.time()
        self.fps_dq = deque([], maxlen=50)
        self.fps = 0.0

        # Set up window and mouse callback for AOI selection
        # cv2.namedWindow("Stick Detection", flags=cv2.WINDOW_GUI_NORMAL)
        # cv2.setMouseCallback("Stick Detection", self.mouse_callback)

        # Main loop: Detection and display
        try:
            while self.running:
                # Get the latest frame

                frame = self.video_stream.get_frame()
                if frame is not None:
                    self.frame_height, self.frame_width = frame.shape[:2]

                    # measure fps
                    now = time.time()
                    self.fps_dq.append(1.0 / (now - self.last_time))
                    self.fps = sum(self.fps_dq) / len(self.fps_dq)
                    self.last_time = now
                else:
                    continue  # Skip if no frame yet

                # Initialize prev_aoi with first valid AOI
                if self.aoi and self.prev_aoi is None:
                    x, y, w, h = self.aoi
                    self.prev_aoi = frame[y : y + h, x : x + w].copy()
                    
                # Perform detection
                if self.detecting:
                    # Check for drop
                    start_time = time.time()
                    changes = self.check_drop(frame)
                    # self.check_drop_cni(frame)
                    end_time = time.time()
                    if self.verbose and changes == True:
                        print(f"Drop Check Time: {((end_time - start_time)*1e3):.3f} ms")
                        if self.stick_drop_command_time:
                            print(f"Elapsed time since drop command: { ((end_time - self.stick_drop_command_time)*1e3):.3f} ms")
                        print(f"With AOI: {self.aoi}")

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
                    cv2.rectangle(frame, (x, y), (x + w, y + h), aoi_color, 2)
                    self.draw_status_tag(frame, x, y, status_tag, aoi_color)

                # Display the frame
                # cv2.imshow("Stick Detection", frame)
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.6
                font_color = (255, 255, 255)  # White text
                thickness = 1
                cv2.putText(
                    frame,
                    f"{self.fps:03.1f}",
                    (20, 20),
                    font,
                    font_scale,
                    font_color,
                    thickness,
                    cv2.LINE_AA,
                )

                # # create QImage and send to widget
                h, w, ch = frame.shape
                img = QImage(frame.data, w, h, ch * w, QImage.Format_BGR888)
                self.updateFrame.emit(img)

        except KeyboardInterrupt:
            print("Stopped by user.")
            self.running = False

        # Cleanup
        finally:
            self.video_stream.release()
            if self.video_source == "ROS2":
                import rclpy

                rclpy.shutdown()

            # cv2.destroyAllWindows()
