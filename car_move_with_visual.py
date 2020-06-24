import time
import cv2
import numpy as np
import threading

from realsense_basic import run
from arduino_connector import setArduinoCar, setArduinoArm


image_shape=np.array((640, 480))
thr = threading.Thread(target=lambda: print("Start Thread"))
thr.start()


def detectAndGo(color_image, depth_image):
    """ Custom detect method """
    # detect
    hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0,100,100])
    upper_red = np.array([10,255,255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    lower_red = np.array([175,100,100])
    upper_red = np.array([190,255,255])
    mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower_red, upper_red))
    mask_xy = np.where(mask)
    if len(mask_xy[0]):
        center = np.flip(np.mean(mask_xy, axis=1))
        width = np.array([50, 50])
    else:
        print(mask_xy)
        center = image_shape / 2
        width = image_shape / 16

    # draw center
    tl = (center - width).astype(np.int)
    br = (center + width).astype(np.int)
    cv2.rectangle(color_image, tuple(tl), tuple(br), (0, 255, 0), 2)

    # get depth
    d = depth_image[tl[0]:br[0], tl[1]:br[1]].mean()

    # walk(Run command in threading)
    global thr
    print(d, center, thr)
    if d > 500 and not thr.is_alive():
        if center[0] > image_shape[0] * 2 / 3:
            thr = threading.Thread(target=setArduinoCar, args=('R', 100, 100))
        elif center[0] < image_shape[0] * 1 / 3:
            thr = threading.Thread(target=setArduinoCar, args=('L', 100, 100))
        else:
            thr = threading.Thread(target=setArduinoCar, args=('F', 100, 100))
        thr.start()

    # plot it
    stacked_mask = np.stack((mask,)*3, axis=-1)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    return np.hstack([depth_colormap, color_image, stacked_mask])


if __name__ == "__main__":
    setArduinoArm(0, 0)
    time.sleep(1)
    setArduinoArm(1, 0)
    time.sleep(1)
    run(detectAndGo)
