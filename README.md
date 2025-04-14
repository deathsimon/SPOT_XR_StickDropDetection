**Introduction**

This Python script is designed for the 6GEM Spot-XR Demo in June, 2025.

Its primary function is to detect a target stick in the video stream and trigger the gripper to close, capturing the falling stick.

In this pixel-diff version, we first select the Area-of-Interest (AOI) using mouse. Once the mean square error (MSE) within the AOI exceeds the given threshold, a signal will be sent out for triggering the succeeding actions.


**How to Use**

Fill out the parameters in *config* and start the script.

Use the mouse to drag the AOI. After an AOI is set, the detection process starts automatically.

Press `q' to exit.

After detecting a fall, the script stops the detection. Press `d' or draw a new AOI to restart the detection.


**Dependency**

numpy - Version: 1.24.2

opencv-python - Version: 4.11.0.86

ultralytics - Version: 8.3.99

websockets - Version: 13.1

scikit-image - Version: 0.21.0

**RTSP**

- Download SimpleRTSP server: wget https://github.com/aler9/rtsp-simple-server/releases/download/v0.16.0/rtsp-simple-server_v0.16.0_linux_amd64.tar.gz
- Extract and start the RTSP Server: RTSP_RTSPADDRESS=(YOUR IP):(YOUR PORT) ./rtsp-simple-server
- Stream webamera: ffmpeg -i /dev/video* -rtsp_transport tcp -c:v libx264 -preset ultrafast -tune zerolatency -b:v 500k -c:a aac -strict experimental -f rtsp rtsp://(YOUR IP):(YOUR PORT)/live -v verbose
- Update the rtsp_url in config to rtsp://(YOUR IP):(YOUR PORT)/live

**TODO**

- Test on 360' video streams

- Further optimize performance
