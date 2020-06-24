import pyrealsense2 as rs
import numpy as np
import cv2


image_shape=np.array((640, 480))


class Camera():
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

    # run(test)

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
        # center, width = detect(color_image)
        tl = (center - width).astype(np.int)
        br = (center + width).astype(np.int)
        cv2.rectangle(color_image, tuple(tl), tuple(br), (0, 255, 0), 2)

        # get depth
        d = depth_image[tl[0]:br[0], tl[1]:br[1]].mean()
        print(d)

        # plot it
        stacked_mask = np.stack((mask,)*3, axis=-1)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        return np.hstack([depth_colormap, color_image, stacked_mask])
        return np.hstack([depth_colormap, color_image])

    run(detectAndGo)
