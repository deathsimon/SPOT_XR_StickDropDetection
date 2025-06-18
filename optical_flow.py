import numpy as np
import cv2 as cv
from scipy.ndimage import measurements
from collections import deque

cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

cap.set(cv.CAP_PROP_FPS, 6)

last_frame = None
last_movement = deque(maxlen=3)
total = 0
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if last_frame is None:
        last_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        continue
 
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    # calculate flow
    flow = cv.calcOpticalFlowFarneback(last_frame, frame, None,
                                           pyr_scale = 0.5,
                                            levels = 3,
                                            winsize = 12,
                                            iterations = 1,
                                            poly_n = 5,
                                            poly_sigma = 1.2,
                                            flags = 0) 
    
    mag_img, pha_img = cv.cartToPolar(flow[..., 0], flow[..., 1], angleInDegrees=True) 
    
    # filter by phase
    #print(mag_img)
    filtered  = np.where(((pha_img < 110) & (pha_img > 60)) & (mag_img > 6.0), mag_img, 0)
    
    last_movement.append(filtered.sum())
    
    if np.all(np.array(last_movement) > 10000):
        total += 1
        print(f"Found something! {total}x")
    
    #print(last_movement)
    #filtered, num = measurements.label(filtered)
    #print(filtered)
    #filtered = filtered.astype(np.float64)
    #filtered /= filtered.max()
    
    
    filtered = cv.addWeighted(filtered, 0.1, frame.astype(np.float32) / 256.0, 0.9, 0)
    
    # Display the resulting frame
    cv.imshow('frame', filtered)
    #cv.imshow('frame', frame)
    if cv.waitKey(1) == ord('q'):
        break
    
    last_frame = frame
 
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
