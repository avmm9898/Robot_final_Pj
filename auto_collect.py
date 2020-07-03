import numpy as np
import json
import time
import cv2

# camera
from realsense_basic import Camera
from test_detect_xyz import getPurpleXYD

# arm
from arm_move import Arm
from arm_inverse_kinematic import linkTransform, DH


class NumpyArrayEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        else:
            return super(NumpyArrayEncoder, self).default(obj)

fout = "test4.json"

try:
    cam = Camera()
    arm = Arm()
    data = []
    """
    for i in range(0, 60 + 1, 20):
        for j in range(180, 120 - 1, -20):
            for m in range(5):
                if 60 <= i + j - 90 - 90 + 20 * m <= 120:
                    k = m * 20
                    for z in range(45, 115 + 1, 10):
                        angle = [z, i, j, k]
                        radangle = np.array(angle) * np.pi / 180
                        T0 =        linkTransform(*DH[0][:3], radangle[0] - np.pi / 2)
                        T1 = T0.dot(linkTransform(*DH[1][:3], radangle[1]))
                        T2 = T1.dot(linkTransform(*DH[2][:3], radangle[2] - np.pi / 2))
                        T3 = T2.dot(linkTransform(*DH[3][:3], radangle[3] + np.pi * 3 / 2))
                        T4 = T3.dot(linkTransform(*DH[4][:3], 0))
                        print(T4[:3, 3])
                        if T4[2, 3] < 0.42 - 0.077:  # baseline from floor
                            # move it
                            arm.moveByAngle(angle, slow=False)
                            print(angle)

                            # search purple
                            color_image = None
                            xyd = None
                            c = 0
                            while xyd is None or c < 10:
                                color_image, depth_image = cam.read()
                                if color_image is None or depth_image is None:
                                    continue
                                c += 1
                                time.sleep(0.1)
                                xyd = getPurpleXYD(color_image, depth_image)

                                # Show images
                                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03),
                                                                   cv2.COLORMAP_JET)
                                images = np.hstack([depth_colormap, color_image])
                                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                                cv2.imshow('RealSense', images)
                                key = cv2.waitKey(100)

                            # get xyz and arm xyz
                            xyz = cam.getXYZ(*xyd)
                            axyz = Arm.xyzFromCamera(*xyz)

                            data.append({
                                'arm':  T4[:3, 3],
                                'angle': angle,
                                'xyd': xyd,
                                'xyz': xyz,
                                'axyz': axyz,
                            })
                            print(len(data))
                            json.dump(data, open(fout, 'w'), cls=NumpyArrayEncoder)
    """

    for i in range(0, 60 + 1, 20):
        for j in range(180, 90 - 1, -20):
            k = 180 - (i + j - 90)
            if k < 0:
                k = 0
            for z in range(45, 115 + 1, 10):
                angle = [z, i, j, k]
                radangle = np.array(angle) * np.pi / 180
                T0 =        linkTransform(*DH[0][:3], radangle[0] - np.pi / 2)
                T1 = T0.dot(linkTransform(*DH[1][:3], radangle[1]))
                T2 = T1.dot(linkTransform(*DH[2][:3], radangle[2] - np.pi / 2))
                T3 = T2.dot(linkTransform(*DH[3][:3], radangle[3] + np.pi * 3 / 2))
                T4 = T3.dot(linkTransform(*DH[4][:3], 0))
                print(T3[:3, 3])
                if T3[2, 3] < 0.42 - 0.077:  # baseline from floor
                    # move it
                    arm.moveByAngle(angle, slow=True)
                    print(angle)
                    # quick review
                    # continue

                    # search purple
                    color_image = None
                    xyd = None
                    c = 0
                    while xyd is None or c < 10:
                        color_image, depth_image = cam.read()
                        if color_image is None or depth_image is None:
                            continue
                        c += 1
                        time.sleep(0.1)
                        xyd = getPurpleXYD(color_image, depth_image)

                        # Show images
                        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03),
                                                           cv2.COLORMAP_JET)
                        images = np.hstack([depth_colormap, color_image])
                        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                        cv2.imshow('RealSense', images)
                        key = cv2.waitKey(100)

                    # get xyz and arm xyz
                    xyz = cam.getXYZ(*xyd)
                    axyz = Arm.xyzFromCamera(*xyz)

                    data.append({
                        'arm':  T4[:3, 3],
                        'angle': angle,
                        'xyd': xyd,
                        'xyz': xyz,
                        'axyz': axyz,
                    })
                    print(len(data))
                    json.dump(data, open(fout, 'w'), cls=NumpyArrayEncoder)
finally:
    cam.stop()
