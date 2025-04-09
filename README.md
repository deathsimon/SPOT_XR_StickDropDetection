**Introduction**

This Python script is designed for the 6GEM Spot-XR Demo in June, 2025.

Its primary function is to detect a target stick in the video stream and trigger the gripper to close, capturing the falling stick.

We utilize a YOLO v11m model to detect the stick's position. If the stick moves vertically beyond a defined threshold, it is considered to be falling.


**How to Use**

Fill out the parameters in *config* and start the script.

Press `q' to exit.

After detecting a fall, the script stops the detection. Press `d' to restart the detection.


**Dependency**

numpy - Version: 1.24.2

opencv-python - Version: 4.11.0.86

ultralytics - Version: 8.3.99

**RTSP**

- Download SimpleRTSP server: wget https://github.com/aler9/rtsp-simple-server/releases/download/v0.16.0/rtsp-simple-server_v0.16.0_linux_amd64.tar.gz
- Extract and start the RTSP Server: RTSP_RTSPADDRESS=(YOUR IP):(YOUR PORT) ./rtsp-simple-server
- Stream webamera: ffmpeg -i /dev/video* -rtsp_transport tcp -c:v libx264 -preset ultrafast -tune zerolatency -b:v 500k -c:a aac -strict experimental -f rtsp rtsp://(YOUR IP):(YOUR PORT)/live -v verbose
- Update the rtsp_url in config to rtsp://(YOUR IP):(YOUR PORT)/live

**TODO**

- The COCO dataset used for training the YOLO model does not include a "stick" class. Possible alternatives:
  - Use 'bottle' (class 39) or 'baseball bat' (class 36) as a proxy
  - Train a custom YOLO model with relevant images

- Evaluate detection latency

- Further optimize performance
