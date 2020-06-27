# main function
import time
import cv2
import numpy as np
import threading

from realsense_basic import run, Camera
from arduino_connector import setArduinoCar, setArduinoArm
from car_move_with_visual import detectAndGo
from arm_move_with_visual import onetime_run, getXYZ


# move arm away
setArduinoArm(0, 0)
time.sleep(1)
setArduinoArm(1, 0)
time.sleep(1)

# move car
run(detectAndGo)

# move arm
onetime_run(getXYZ)
