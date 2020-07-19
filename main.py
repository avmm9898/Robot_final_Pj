import time
from functools import partial

from realsense_basic import Camera
from car_move_with_visual import detectAndApproach, carApproach, carApproachMinor
from arm_move_with_visual import yoloDetect, detectAndInsert
from playYOLO import YOLO
from arm_move import Arm
from control import continueRun

# load
platform_net = YOLO("platform_tiny", 0.4)
hole_net     = YOLO("yolov3_hole", 0.8)

try:
    print("Start")
    cam = Camera()
    arm = Arm()
    arm.hide()
    time.sleep(1)

    print("Approach")
    continueRun(partial(detectAndApproach,
                        arm=arm,
                        func_detect=platform_net.detect,
                        func_approach=carApproach),
                cam=cam)

    print("Approach Minor")
    continueRun(partial(detectAndApproach,
                        arm=arm,
                        func_detect=hole_net.detect,
                        func_approach=carApproachMinor),
                cam=cam)

    print("Insert")
    detectAndInsert(cam, arm, partial(yoloDetect, hole_net.detect),
                    multiple=True)

finally:
    cam.stop()
