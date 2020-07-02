import time
import cv2
import numpy as np
import threading
from functools import partial

from realsense_basic import run
from arduino_connector import setArduinoCar, setArduinoArm
from playYOLO import YOLO

# init
image_shape = np.array((640, 480))
thr = threading.Thread(target=lambda: print("Start Thread"))
thr.start()
depth_moving_window = [999] * 5  # the array for moving average


def example_detect(color_image, depth_image):
    """ Example detecting code """
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
        width = np.array([20, 20])
    else:
        print(mask_xy)
        center = image_shape / 2
        width = image_shape / 16

    # draw center
    tl = (center - width).astype(np.int)
    br = (center + width).astype(np.int)
    cv2.rectangle(color_image, tuple(tl), tuple(br), (0, 255, 0), 2)
    return [center]


def detectAndGo(func_detect, color_image, depth_image):
    """ Custom detect method """
    # detect
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.3), cv2.COLORMAP_JET)
    centers = func_detect(color_image)
    IMG_x=320
    IMG_y=240

    if len(centers)>0:
        for i in centers:
            IMG_x=int(i[0])
            IMG_y=int(i[1])

    # get depth
    d = depth_image[IMG_x-50:IMG_x+50, IMG_y-50:IMG_y+50].mean()

    global depth_avg
    depth_avg = depth_avg[1:] + [d]
    d=np.mean(depth_avg)

    # walk(Run command in threading)
    global thr
    print(d, (IMG_x, IMG_y), thr)

    is_ok = False  # Terminated
    if not thr.is_alive():
        
        if d <= 380:
            is_ok = True
        else:
            if IMG_x > image_shape[0] / 2 + 30:
                thr = threading.Thread(target=setArduinoCar, args=('R', 150, 1))
            elif IMG_x < image_shape[0] / 2 - 70:
                thr = threading.Thread(target=setArduinoCar, args=('L', 150, 1))
            else:
                thr = threading.Thread(target=setArduinoCar, args=('F', 120, 2))
            thr.start()

    # plot it
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    return is_ok, np.hstack([depth_colormap, color_image])

def MinorApproach(func_detect, color_image, depth_image):
    # detect
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.3), cv2.COLORMAP_JET)
    centers = func_detect(color_image)
    mean_x=0
    mean_y=0

    if len(centers)>0:
        for i in centers:
            mean_x+=int(i[0]/len(centers))
            mean_y+=int(i[1]/len(centers))

    # get depth
    d = depth_image[IMG_x-20:IMG_x+20, IMG_y-20:IMG_y+20].mean()

    global depth_moving_window
    depth_moving_window = depth_moving_window[1:] + [d]
    d=np.mean(depth_moving_window)

    # walk(Run command in threading)
    global thr
    print(d, (mean_x, mean_y), thr)

    is_ok = False  # Terminated
    if not thr.is_alive():
        
        if d <= 400 and d >=350:
            is_ok = True
        elif d <= 350:
            thr = threading.Thread(target=setArduinoCar, args=('B', 120, 1))
            thr.start()
        else:
            if mean_x > image_shape[0] / 2 + 30:
                thr = threading.Thread(target=setArduinoCar, args=('R', 150, 1))
            elif mean_y < image_shape[0] / 2 - 70:
                thr = threading.Thread(target=setArduinoCar, args=('L', 150, 1))
            else:
                thr = threading.Thread(target=setArduinoCar, args=('F', 120, 1))
            thr.start()

    # plot it
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    return is_ok, np.hstack([depth_colormap, color_image])



if __name__ == "__main__":
    net = YOLO("platform_tiny", 0.3)
    # net = YOLO("yolov3_hole", 0.8)
    setArduinoArm(0, 0)
    time.sleep(1)
    setArduinoArm(1, 0)
    time.sleep(1)
    run(partial(detectAndGo, net.detect))
    # run(partial(detectAndGo, example_detect))
