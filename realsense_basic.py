import pyrealsense2 as rs
import numpy as np
import cv2


image_shape=np.array((640, 480))


class Camera():
    """
    The usage:
    cam = Camera()
    color_image, depth_image = cam.read()
    """

    def __init__(self, image_shape=(640, 480), fps=30):
        self.pipeline = None
        self.aligned = None
        self.image_shape = image_shape

        """ Initial and setup the realsense camera """
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, image_shape[0], image_shape[1], rs.format.bgr8, fps)
        config.enable_stream(rs.stream.depth, image_shape[0], image_shape[1], rs.format.z16,  fps)

        # align
        self.aligned = rs.align(rs.stream.color)

        # Start streaming
        self.pipeline.start(config)


    def read(self):
        """ Read Image, Return RGB image and depth image """
        # Read and align
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.aligned.process(frames)

        # Convert images to numpy arrays
        color_image = np.asanyarray(aligned_frames.get_color_frame().get_data())
        depth_image = np.asanyarray(aligned_frames.get_depth_frame().get_data())
        # color_image = np.asanyarray(frames.get_color_frame().get_data())
        # depth_image = np.asanyarray(frames.get_depth_frame().get_data())
        return color_image, depth_image

    def stop(self):
        print("Stop camera")
        self.pipeline.stop()


def run(func):
    """ Run forever with func """
    try:
        camera = Camera()
        while True:
            color_image, depth_image = camera.read()
            if color_image is None:
                return
            images = func(color_image, depth_image)

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            key = cv2.waitKey(100)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        camera.stop()


if __name__ == "__main__":
    def test(color_image, depth_image):
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.3), cv2.COLORMAP_JET)
        return np.hstack((depth_colormap, color_image))
    run(test)
