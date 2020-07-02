"""
The test script is to collect the
* XYZ from arm inverse 
* XYZ from camera
* XYZ from arm base
data.

It move the arm by x y z in `test` file and
move the arm away to collect vision xyz and it's transform.
After click s, it will save the data into `test1` file.
"""

import time
import cv2
import numpy as np

# XYZ
from test_detect_xyz import getPurpleXYD
from arm_move import Arm

# camera
from realsense_basic import Camera


pre_xyz = [0, 0, 0]
try:
    # init
    cam = Camera()
    arm = Arm()
    arm.hide()

    while True:
        # read
        color_image, depth_image = cam.read()
        if color_image is None or depth_image is None:
            continue

        want_xyz = np.array(open("test").read().split(' '), dtype=np.float)
        if np.any(pre_xyz != want_xyz):
            print("Go", want_xyz)
            try:
                pre_xyz = want_xyz
                arm.moveByXYZ(*want_xyz)
            except ValueError as e:
                print(e)

        xyd = getPurpleXYD(color_image, depth_image)
        if xyd is None:
            continue

        xyz = cam.getXYZ(*xyd)
        axyz = Arm.xyzFromCamera(*xyz)
        print("arm xyz", f"{axyz[0]:.03f} {axyz[1]:.03f} {axyz[2]:.03f}")

        # Show images
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03),
                                           cv2.COLORMAP_JET)
        images = np.hstack([depth_colormap, color_image])
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        key = cv2.waitKey(100)

        # more away arm
        if key & 0xFF == ord('0'):
            arm.moveByAngle([0, 90, 90, 90])
            arm.hide()

        if key & 0xFF == ord('s'):
            open("test1", "a").write(str(np.stack([want_xyz, xyz, axyz]).flatten()))
            print("Save")


        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

finally:
    cam.stop()
