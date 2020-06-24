import time
import cv2
import numpy as np

from realsense_basic import Camera
from arduino_connector import setArduinoArm

cam = Camera()
color_image, depth_image = cam.read()

# calculate x y z r p y
# ...

ths = [90, 90, 90, 90]
for i in range(4):
    setArduinoArm(i, ths[i])
