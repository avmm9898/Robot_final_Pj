import time
import cv2
import numpy as np
import pyrealsense2 as rs

from arm_move import Arm


def purple_detect(color_image):
    """ An example for detecting purple things and output bounding box """
    # get purple color
    hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
    lower_purple = np.array([120, 20, 50])
    upper_purple = np.array([160, 80, 185])
    mask = cv2.inRange(hsv, lower_purple, upper_purple)

    # group purple area
    mask1 = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15)))
    _, _, stats, centroids = cv2.connectedComponentsWithStats(mask1, connectivity=8)

    # debug: cannot find things
    if len(stats) <= 1 or stats[1 + np.argmax(stats[1:, 4]), -1] < 200:
        if False:
            import matplotlib.pyplot as plt
            hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
            plt.imshow(hsv)
            plt.show()
        return None, None

    # find max area of purple
    # print(stats)
    idx = 1 + np.argmax(stats[1:, 4])
    box = stats[idx]
    # center = centroids[idx].astype(np.int)
    center = np.array([box[0] + box[2] // 2, box[1] + box[3] // 2])
    print("center and box", center, box)
    return center, box


def getPurpleXYD(color_image, depth_image):
    # detect
    center, box = purple_detect(color_image)
    if center is None:
        return None

    # plot bounding box
    cv2.rectangle(color_image, tuple(box[:2]), tuple(box[:2] + box[2:4]), (0, 255, 0), 2)
    cv2.rectangle(color_image, tuple(center - 1), tuple(center + 1), (255, 0, 0), 2)

    # get real xyz
    d = depth_image[center[1]-1:center[1]+2, center[0]-1:center[0]+2].mean()
    return [center[1], center[0], d]


def getPurpleXYZ(cam, color_image, depth_image):
    """ Get object XYZ from Image """
    # init
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03),
                                       cv2.COLORMAP_JET)
    images = np.hstack([depth_colormap, color_image])

    # get xyd and xyz
    xyd = getPurpleXYD(color_image, depth_image)
    if xyd is None:
        return False, images
    xyz = cam.getXYZ(*xyd)

    print("xyd", f"{xyd[0]:04.0f} {xyd[1]:04.0f} {xyd[2]:04.0f}",
          "xyz", f"{xyz[0]:04.0f} {xyz[1]:04.0f} {xyz[2]:04.0f}")

    # get arm xyz
    axyz = Arm.xyzFromCamera(*xyz)
    print("arm xyz", f"{axyz[0]:.03f} {axyz[1]:.03f} {axyz[2]:.03f}")

    # get arm angle
    try:
        angle = Arm.xyz2Angle(*axyz)
        print("Angle", angle)
    except ValueError as e:
        print(e)

    images = np.hstack([depth_colormap, color_image])
    return False, images


if __name__ == "__main__":
    from control import continueRun
    continueRun(getPurpleXYZ)
