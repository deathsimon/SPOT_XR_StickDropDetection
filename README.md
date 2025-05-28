## 6GEM Spot-XR Stick-Drop Demo (June 2025)

This Python script powers the 6GEM Spot-XR stick-drop demonstration, scheduled for June 2025.

Its job is to detect when a stick begins to fall in a live video stream and command the gripper to snap shut before the object hits the ground.

### Key Features

- **Flexible Video Input**  
  Supports both local cameras and remote RTSP streams.
- **Interactive AOI Selection**  
  Click and drag with your mouse to define an Area of Interest (AOI) around the stick’s resting position.
- **Dual Detection Methods**  
  - **Mean Squared Error (MSE)**  
  - **Structural Similarity Index (SSIM)**  
  Compare the AOI in each incoming frame against a static reference.  
- **Threshold-Driven Trigger**  
  When the computed MSE or SSIM score crosses its user-configurable threshold, the script instantly sends a “close” command to the gripper.

### How It Works
1. **Initialize Stream**  
   Connect to your chosen video source.
2. **Select AOI**
   Drag the AoI around the stick using mouse to capture the reference frame.   
3. **Monitor & Compare**  
   For each new frame, the script computes MSE or SSIM over the AOI versus the reference frame.
4. **Actuate Gripper**  
   If the difference exceeds the predefined threshold, the gripper-close command will be issued, ideally securing the stick in mid-air.
   If it's too late, please consider lowering the threshold.
5. **Restart Detection**
   After detecting a fall, the script stops the detection. Press `d' or draw a new AOI to restart the detection.
6. **Exit the Script**
   Press `q' to exit.

***Other commands for Spot:***

- 'r': reset to the default setting for catching
- 'o': open the gripper
- 'c': close the gripper
- 's': stow the arm and sit down

### Dependency

- numpy - Version: 1.24.2
- opencv-python - Version: 4.11.0.86
- ultralytics - Version: 8.3.99
- websockets - Version: 13.1
- scikit-image - Version: 0.21.0

### RTSP

- Download SimpleRTSP server: wget https://github.com/aler9/rtsp-simple-server/releases/download/v0.16.0/rtsp-simple-server_v0.16.0_linux_amd64.tar.gz
- Extract and start the RTSP Server: RTSP_RTSPADDRESS=(YOUR IP):(YOUR PORT) ./rtsp-simple-server
- Stream webamera: ffmpeg -i /dev/video* -rtsp_transport tcp -c:v libx264 -preset ultrafast -tune zerolatency -b:v 500k -c:a aac -strict experimental -f rtsp rtsp://(YOUR IP):(YOUR PORT)/live -v verbose
- Update the rtsp_url in config to rtsp://(YOUR IP):(YOUR PORT)/live


