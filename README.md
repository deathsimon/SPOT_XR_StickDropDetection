# 6GEM: Real-time Networked Robotics Control with Embedded ROS Demo

This repository contains the software for a demonstration of real-time networked robotics, presented at the **8th 6GEM General Assembly at TU Dortmund on June 17-18, 2025**.

The project showcases the critical impact of network latency on remote robotic control. In our demonstration, a falling stick is monitored by one robot (the "Xplorer") via a video stream processed in an edge cloud. A second robot (the "Rescuer") must react in time to catch the stick based on commands sent over the network. The height at which the stick is caught serves as a direct and tangible measure of the end-to-end latency of the entire control loop.

## Demo Video

The following video was recorded during the live demonstration at the event.

[![Spot-XR Stick-Drop Demo](https://img.youtube.com/vi/5mrTymGEN-Q/0.jpg)](https://youtube.com/shorts/5mrTymGEN-Q?feature=share)

## Key Features

- **Flexible Video Input**: Supports local cameras, remote RTSP streams, and ROS2 topics.
- **Interactive AOI Selection**: Easily define an Area of Interest (AOI) by clicking and dragging with the mouse.
- **Multiple Detection Methods**: Choose from three detection algorithms:
  - **Mean Squared Error (MSE)**: Measures the average squared difference between the pixels of two images.
  - **Structural Similarity Index (SSIM)**: Compares the structural information of two images.
  - **Optical Flow**: Tracks the motion of features between frames to detect downward movement.
- **Real-time Configuration**: Adjust detection parameters on-the-fly using the `hot_load_config` file.
- **GUI Dashboard**: A user-friendly interface for monitoring the video feed and system status.

## Getting Started

### Prerequisites

- Python 3.8+
- A running Spot robot with its WebSocket server enabled.

### Installation

1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   cd SPOT_XR_Detection_Stick
   ```

2. **Install the dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

3. **Configure the application:**
   - Open the `config` file and set the `video_source` (either `RTSP`, `Camera`, or `ROS2`).
   - If using RTSP or a specific camera, update the `rtsp_url` or `camera_index` accordingly.
   - Set the `IP` and `port` of the Spot robot's WebSocket server.

4. **Run the application:**
   ```bash
   python dashboard.py
   ```

## How It Works

1. **Initialization**: The application connects to the specified video source and the Spot robot.
2. **AOI Selection**: The user draws a rectangle on the video feed to define the Area of Interest (AOI) where the stick is located.
3. **Detection**: The application continuously monitors the AOI for changes. Depending on the selected `fall_detection_method`, it uses MSE, SSIM, or Optical Flow to detect the stick's movement.
4. **Gripper Actuation**: When the detection algorithm determines that the stick is falling, it sends a "close" command to the Spot robot's gripper.
5. **Reset**: After a fall is detected, the system can be reset for the next attempt.

## Configuration

The application uses two configuration files:

- **`config`**: This file contains the main settings for the application, such as the video source, robot IP address, and default detection parameters. These settings are loaded once at startup.
- **`hot_load_config`**: This file allows you to adjust detection parameters in real-time. Any changes saved to this file will be immediately applied by the application.

### `hot_load_config` Parameters

| Parameter                 | Description                                                                 |
| ------------------------- | --------------------------------------------------------------------------- |
| `fall_detection_method`   | The detection method to use: `0` for MSE, `1` for SSIM, `2` for Optical Flow. |
| `MSE_threshold`           | The MSE threshold for detecting a fall.                                     |
| `SSIM_change_threshold`   | The SSIM change threshold for detecting a fall.                             |
| `of_angle_tolerance`      | The angle tolerance for the optical flow algorithm.                         |
| `of_magnitude_thres`      | The magnitude threshold for the optical flow algorithm.                     |
| `stored_frames`           | The number of frames to store for comparison.                               |
| `required_frames`         | The number of consecutive frames that must exceed the threshold to trigger a fall. |

## Controls

| Key | Action                               |
| --- | ------------------------------------ |
| `d` | Toggle detection on/off.             |
| `r` | Reset the Spot Arm to its default position. |
| `o` | Open the gripper.                    |
| `c` | Close the gripper.                   |
| `s` | Stow the arm and sit down.           |
| `1` | Manually trigger the stick to drop.  |
| `2` | Manually reset the stick holder.     |
| `x` | Save the current AOI to the `hot_load_config` file. |
| `z` | Reload the settings from the `hot_load_config` file. |
| `,` | Set predictive scheduling.           |
| `.` | Set reactive scheduling.             |
| `f` | Toggle fullscreen mode.              |
| `q` | Quit the application.                |

## Project Structure

```
.
├── dashboard.py            # Main application entry point (GUI)
├── stick_detection.py      # Core detection logic
├── Spot_arm.py             # WebSocket client for Spot Arm control
├── ApiManager.py           # Defines the API for the Spot robot
├── holdercontrol.py        # Server-side control for the stick holder
├── rodholderclient.py      # Client for the stick holder
├── video_source_*.py       # Video source implementations
├── config                  # Main configuration file
├── hot_load_config         # Hot-reloadable configuration file
├── dashboard_v2.ui         # UI layout file
└── requirements.txt        # Python dependencies
```

## RTSP Setup (Optional)

If you are using an RTSP stream as the video source, you can use the following steps to set up a simple RTSP server.

1.  **Download the RTSP server:**
    ```bash
    wget https://github.com/aler9/rtsp-simple-server/releases/download/v0.16.0/rtsp-simple-server_v0.16.0_linux_amd64.tar.gz
    ```
2.  **Extract and start the server:**
    ```bash
    RTSP_RTSPADDRESS=<YOUR_IP>:<YOUR_PORT> ./rtsp-simple-server
    ```
3.  **Stream your webcam:**
    ```bash
    ffmpeg -i /dev/video* -rtsp_transport tcp -c:v libx264 -preset ultrafast -tune zerolatency -b:v 500k -c:a aac -strict experimental -f rtsp rtsp://<YOUR_IP>:<YOUR_PORT>/live -v verbose
    ```
4.  **Update the `rtsp_url` in the `config` file.**


