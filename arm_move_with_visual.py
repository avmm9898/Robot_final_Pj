import time
import cv2
import numpy as np

from arm_move import Arm


def yoloDetect(func_detect, color_image, depth_image):
    """ Given a yolo detect function and output Image x y d """
    # detect by specific yolo model
    centers = func_detect(color_image)
    if len(centers) == 0:
        return None

    # the center of holes
    mean_x = int(np.mean([i[0] for i in centers]))
    mean_y = int(np.mean([i[1] for i in centers]))

    # get depth
    d = depth_image[mean_y-20:mean_y+20, mean_x-20:mean_x+20].mean()

    # get location
    xyds = [[i[1], i[0], d] for i in centers]
    print(xyds)
    return xyds


def detectAndInsert(cam, arm, func, multiple=False):
    """
    Detect Image by custom function and Insert by arm.

    Set multiple=True if you want to insert multiple times
    """
    # wait for stable the image, get the last one
    xyds = []
    c = 0
    while c < 10:
        color_image, depth_image = cam.read()
        if color_image is None or depth_image is None:
            time.sleep(0.1)
            return
        xyds = func(color_image, depth_image)
        if xyds:
            c += 1

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03),
                                           cv2.COLORMAP_JET)
        images = np.hstack([depth_colormap, color_image])
        cv2.imshow('RealSense', images)
        key = cv2.waitKey(10)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
        time.sleep(0.1)

    # Move arm
    arm.reset()
    if not multiple:
        xyds = [xyds]

    for xyd in xyds:
        if xyd is None:
            continue

        xyz = cam.getXYZ(*xyd)
        axyz = Arm.xyzFromCamera(*xyz)
        print("arm xyz", f"{axyz[0]:.03f} {axyz[1]:.03f} {axyz[2]:.03f}")
        try:
            # arm.moveByXYZ(*axyz)
            arm.moveByXYZthroughX(*axyz)
            time.sleep(3)
            arm.reset()
        except ValueError as e:
            print(e)


if __name__ == "__main__":
    from test_detect_xyz import getPurpleXYD
    from playYOLO import YOLO
    from functools import partial
    from realsense_basic import Camera

    try:
        cam = Camera()
        arm = Arm()
        arm.hide()
        # hole_net = YOLO("yolov3_hole", 0.8)
        # detectAndInsert(cam, arm, partial(yoloDetect, hole_net.detect)), multiple=True)
        detectAndInsert(cam, arm, getPurpleXYD)
    finally:
        cam.stop()
