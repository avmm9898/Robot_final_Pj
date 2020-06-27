import time
import cv2
import numpy as np
import pyrealsense2 as rs

from realsense_basic import Camera
# from arduino_connector import setArduinoArm
from arm_inverse_kinematic import xyz2Armangle


debug = True


def purple_detect(color_image):
    """ An example for detecting purple things and output bounding box """
    # get purple color
    hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
    lower_purple = np.array([120, 20, 130])
    upper_purple = np.array([160, 40, 185])
    mask = cv2.inRange(hsv, lower_purple, upper_purple)

    # group purple area
    mask1 = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15)))
    _, _, stats, centroids = cv2.connectedComponentsWithStats(mask1, connectivity=8)

    # debug: cannot find things
    if len(stats) <= 1:
        if debug:
            import matplotlib.pyplot as plt
            hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
            plt.imshow(hsv)
            plt.show()
        return None, None

    # find max area of purple
    idx = 1 + np.argmax(stats[1:, 4])
    box = stats[idx]
    center = centroids[idx].astype(np.int)
    # print(center, box)
    return center, box


def getXYZ(cam, color_image, depth_image):
    """ Get object XYZ from Image """
    center, box = purple_detect(color_image)
    if center is None:
        return None

    # get real xyz
    d = depth_image[center[1]-1:center[1]+2, center[0]-1:center[0]+2].mean()
    xyz = cam.getXYZ(center[1], center[0], d)

    # debug
    if debug:
        print("intr_depth", xyz)

    # plot bounding box
    cv2.rectangle(color_image, tuple(box[:2]), tuple(box[:2] + box[2:4]), (0, 255, 0), 2)
    cv2.rectangle(color_image, tuple(center - 1), tuple(center + 1), (255, 0, 0), 2)

    return xyz


def moveArmByXYZ(xyz):
    """ Give xyz then move arm """
    # get angle for arm
    ths = xyz2Armangle(*xyz)

    # set arm angle
    for i in range(4):
        setArduinoArm(i, ths[i])


def continue_run(func):
    """ Run forever for debug """
    try:
        cam = Camera()

        while True:
            color_image, depth_image = cam.readWithIntrinsics()
            if color_image is None or depth_image is None:
                return
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            xyz = func(cam, color_image, depth_image)

            # draw box on it
            images = np.hstack([depth_colormap, color_image])

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            key = cv2.waitKey(100)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        cam.stop()


def onetime_run(func):
    """ Run one time for real world """
    try:
        cam = Camera()
        xyzs = []

        # wait for stable the image
        for i in range(10):
            color_image, depth_image = cam.readWithIntrinsics()
            if color_image is None or depth_image is None:
                continue
            time.sleep(0.1)

            # collect xyz
            xyz = func(cam, color_image, depth_image)
            if xyz is None:
                continue
            xyzs.append(xyz)

            # debug
            if debug:
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                images = np.hstack([depth_colormap, color_image])
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense', images)
                key = cv2.waitKey(1)

        # get xyz and move the arm
        xyz = np.mean(xyzs, axis=0)
        if not debug:
            print(xyz)
        moveArmByXYZ(xyz)

        # Show images
        if debug:
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                cam.stop()
                return

    finally:
        cam.stop()


if __name__ == "__main__":
    onetime_run(getXYZ)
    # continue_run(getXYZ)
