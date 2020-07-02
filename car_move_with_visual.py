import cv2
import numpy as np
import threading

from arduino_connector import setArduinoCar
from config import image_shape

# init
thr = threading.Thread(target=lambda: print("Start Thread"))
thr.start()
depth_moving_window = [999] * 5  # the array for moving average


def carApproach(IMG_x, IMG_y, d):
    """ Custom deceision """
    # init
    global thr
    if thr.is_alive():
        return False
    is_ok = False

    if d <= 600:
        is_ok = True
    else:
        if IMG_x > image_shape[0] / 2 + 30:
            thr = threading.Thread(target=setArduinoCar, args=('R', 150, 1))
        elif IMG_x < image_shape[0] / 2 - 30:
            thr = threading.Thread(target=setArduinoCar, args=('L', 150, 1))
        else:
            thr = threading.Thread(target=setArduinoCar, args=('F', 120, 2))
        thr.start()

    return is_ok


def carApproachMinor(IMG_x, IMG_y, d):
    """ Custom deceision1 """
    # init
    global thr
    if not thr.is_alive():
        return False
    is_ok = False

    if d <= 510 and d >= 480:
        while(True):
            if mean_x > image_shape[0] / 2 + 30:
                thr = threading.Thread(target=setArduinoCar, args=('R', 125, 1))
            elif mean_y < image_shape[0] / 2 - 30:
                thr = threading.Thread(target=setArduinoCar, args=('L', 125, 1))
            else:
                break
        is_ok = True
        print("true")
    elif d < 480:
        thr = threading.Thread(target=setArduinoCar, args=('B', 120, 1))
        thr.start()
    else:
        """
        if mean_x > image_shape[0] / 2 + 50:
            thr = threading.Thread(target=setArduinoCar, args=('R', 150, 1))
        elif mean_y < image_shape[0] / 2 - 50:
            thr = threading.Thread(target=setArduinoCar, args=('L', 150, 1))
        else:"""
        thr = threading.Thread(target=setArduinoCar, args=('F', 120, 1))
        thr.start()

    return is_ok


def detectAndApproach(cam, color_image, depth_image,
                      arm, func_detect, func_approach):
    """ Detect the object and approach it """
    # detect
    centers = func_detect(color_image, depth_image)

    # Average all center of bounding boxes
    if len(centers):
        IMG_x = int(np.mean([i[0] for i in centers]))
        IMG_y = int(np.mean([i[1] for i in centers]))
    else:
        IMG_x, IMG_y = image_shape // 2

    # get depth
    d = depth_image[IMG_y-50:IMG_y+50, IMG_x-50:IMG_x+50].mean()

    # average depth
    global depth_moving_window
    depth_moving_window = depth_moving_window[1:] + [d]
    d = np.mean(depth_moving_window)

    # walk(Run command in threading)
    print(d, (IMG_x, IMG_y), thr)
    is_ok = func_approach(IMG_x, IMG_y, d)

    # plot it
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03),
                                       cv2.COLORMAP_JET)
    return is_ok, np.hstack([depth_colormap, color_image])


if __name__ == "__main__":
    from functools import partial
    from test_detect_xyz import getPurpleXYD
    from playYOLO import YOLO
    from control import continueRun
    from arm_move import Arm
    import time

    def example_detect(color_image, depth_image):
        """ Example detecting code """
        xyd = getPurpleXYD(color_image, depth_image)
        if xyd is None:
            return []
        return [[xyd[1], xyd[0]]]

    arm = Arm()
    arm.hide()
    time.sleep(1)
    # net = YOLO("platform_tiny", 0.3)
    # net = YOLO("yolov3_hole", 0.8)
    continueRun(partial(detectAndApproach,
                        arm=arm,
                        func_detect=example_detect,
                        func_approach=carApproach))
