**Introduction**

This Python script is designed for the 6GEM Spot-XR Demo in June, 2025.

Its primary function is to detect a target stick in the video stream and trigger the gripper to close, capturing the falling stick.

We utilize a YOLO v11m model to detect the stick's position. If the stick moves vertically beyond a defined threshold, it is considered to be falling.

**TODO**

- The COCO dataset used for training the YOLO model does not include a "stick" class. Possible alternatives:
  - Use 'bottle' (class 39) or 'baseball bat' (class 36) as a proxy
  - Train a custom YOLO model with relevant images

- Evaluate detection latency

- Further optimize performance
