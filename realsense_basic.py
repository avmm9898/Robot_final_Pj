import pyrealsense2 as rs
import numpy as np
import cv2

# Custom data
image_shape = np.array((640, 480), dtype=np.int)
scale = 170 / 640
focus = 150
fps = 30


class Camera():
    """
    The usage:
    cam = Camera()
    color_image, depth_image = cam.read()
    xyz = cam.getXYZ(x, y, d)
    """

    def __init__(self):
        self.pipeline = None
        self.aligned = None
        self.image_shape = image_shape

        """ Initial and setup the realsense camera """
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, image_shape[0], image_shape[1],
                             rs.format.bgr8, fps)
        config.enable_stream(rs.stream.depth, image_shape[0], image_shape[1],
                             rs.format.z16,  fps)

        # align
        self.aligned = rs.align(rs.stream.color)

        # Start streaming
        self.pipeline.start(config)

        print("Start Camera")

    def read(self):
        """ Read Image, Return RGB image and depth image """
        # Read and align
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.aligned.process(frames)

        # Convert images to numpy arrays
        color_image = np.asanyarray(aligned_frames.get_color_frame().get_data())
        depth_image = np.asanyarray(aligned_frames.get_depth_frame().get_data())
        return color_image, depth_image

    def readWithIntrinsics(self):
        """ Not work. Use this before getDepth """
        # read
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        # color_frame = frames.get_color_frame()

        # save intrinsics
        self.depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()

        # Convert images to numpy arrays
        aligned_frames = self.aligned.process(frames)
        color_image = np.asanyarray(aligned_frames.get_color_frame().get_data())
        depth_image = np.asanyarray(aligned_frames.get_depth_frame().get_data())
        return color_image, depth_image

    def getXYZ(self, x, y, d):
        """ Get realworld xyz from Image x,y,d """
        # calibrated transformation in 2020-06-30
        cy = (x - image_shape[1] / 2) * -scale
        cx = (y - image_shape[0] / 2) * scale
        z = d
        x = cx * z / focus
        y = cy * z / focus

        # This build-in method is not correct
        # Used this before readWithIntrinsics
        # return rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [x, y], d)
        return [x, y, z]

    def stop(self):
        print("Stop camera")
        self.pipeline.stop()


if __name__ == "__main__":
    # init camera
    camera = Camera()

    # Run forever
    print("Enter q to exit")
    while True:
        # read
        color_image, depth_image = camera.read()
        if color_image is None:
            continue

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.3),
                                           cv2.COLORMAP_JET)
        cv2.imshow('RealSense', np.hstack((depth_colormap, color_image)))
        key = cv2.waitKey(10)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
    camera.stop()
