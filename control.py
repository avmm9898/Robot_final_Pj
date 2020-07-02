from realsense_basic import Camera
import cv2
import time


def continueRun(func, cam=False):
    """ Run forever with func """
    try:
        # init camera
        if not cam:
            camera = Camera()
        else:
            camera = cam

        # forever
        while True:
            # read 
            color_image, depth_image = camera.read()
            if color_image is None:
                return
            ok, images = func(camera, color_image, depth_image)
            if ok:  # A flag to break
                break

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            key = cv2.waitKey(10)
            # time.sleep(0.1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        if not cam:
            camera.stop()


def continueRunSimple(func):
    """ Run forever with func no camera instance as input """
    def wrap(a, b, c):
        return func(b, c)
    continueRun(wrap)
