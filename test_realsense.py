## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
image_shape = (640, 480)
fps = 30
config.enable_stream(rs.stream.color, image_shape[0], image_shape[1], rs.format.bgr8, fps)
config.enable_stream(rs.stream.depth, image_shape[0], image_shape[1], rs.format.z16,  fps)
aligned = rs.align(rs.stream.color)

# Start streaming
pipeline.start(config)

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        aligned_frames = aligned.process(frames)

        # Convert images to numpy arrays
        color_image = np.asanyarray(aligned_frames.get_color_frame().get_data())
        depth_image = np.asanyarray(aligned_frames.get_depth_frame().get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Stack both images horizontally
        images = np.hstack((color_image, depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        key = cv2.waitKey(30)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

finally:

    # Stop streaming
    pipeline.stop()
