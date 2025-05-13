import cv2 as cv
import numpy as np
from pseyepy import Camera

cams = Camera(fps=60, resolution=Camera.RES_LARGE)

# Discard the first frame
cams.read()

cam_index = 0
frame, timestamp = cams.read(cam_index)
cams.end()

cv.imshow("PS3 EYE Image Debug", frame)
cv.waitKey(0)
cv.destroyAllWindows()


