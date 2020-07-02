import numpy as np
import time
from arduino_connector import setArduinoArm
from arm_inverse_kinematic import xyz2angle, rotY, rotZ


class Arm:
    def __init__(self):
        self.init_up = [90, 90, 90, 90]
        self.init_down = [0, 0, 90, 90]
        self.prev_angle = self.init_up
        self.moveByAngle(self.prev_angle)

    def reset(self):
        """ Reset to init position """
        self.moveByAngle(self.init_up)

    def hide(self):
        """ Reset to init position """
        self.moveByAngle(self.init_down)

    def __del__(self):
        """ End with init position """
        self.moveByAngle(self.init_up)

    @classmethod
    def xyzFromCamera(cls, x, y, z):
        """
        Read XYZ from Camera to XYZ from Arm base
        """
        # Transformartion from camera frame to realworld frame
        b = -0.255  # x component of arm base, measured: -.2325
        w =  0.020  # y component of arm base, measured: .0235
        h =  0.265  # z component of arm base, measured: .2975
        cam_angle = 7  # camera yoll angle
        th_y = cam_angle * np.pi / 180 - 1 * np.pi / 2 # rotation about x axis
        th_z = -np.pi / 2                              # rotation about z axis
        T_AC = np.zeros([4, 4])
        T_AC[:3, :3] = rotY(th_y).dot(rotZ(th_z))
        T_AC[:, 3] = [b, w, h, 1]

        # milimeter to meter
        P_TC = np.array([x / 1000, y / 1000, -z / 1000, 1])  # position vector of target in carema frame

        # transform it. now it is realworld frame
        P_AT = T_AC.dot(P_TC)

        # transfer real world frame to Arm frame(It is equal if ideal)
        # y -= 0.03
        # z -= 0.09
        return P_AT[0], P_AT[1], P_AT[2]

    @classmethod
    def xyz2Angle(cls, x, y, z):
        """
        Given real world frame x y z and calculate needed angle in degree
        """

        # Prefer 45 degree downward, and allow 0-90 degree downward
        down_angles = np.stack([np.linspace(-np.pi / 4, -np.pi / 2, num=10),
                                np.linspace(-np.pi / 4,          0, num=10)]).T.flatten()
        for down_angle in down_angles:
            # main calculate function
            angle = xyz2angle(x, y, z, last_angle=down_angle)

            # The real arm init angle different from baseline
            angle[0] += np.pi / 2
            angle[2] += np.pi / 2
            angle[3] -= np.pi * 3 / 2
            angle = np.mod(angle, 2 * np.pi)

            # Return the angle if the angle is available
            if np.any(angle > np.pi) or np.any(np.isnan(angle)):
                # cannot solve
                pass
            else:
                # Success, rad to deg to int
                return np.round(angle * 180 / np.pi).astype(np.int)

        # Not possible solution
        raise ValueError(str(angle) + "is out of working space")

    def moveByAngle(self, angles, slow=True, init_angle=None):
        """ Move the arm by angle in degree """
        if init_angle is None:
            init_angle = self.prev_angle

        # Directly move
        if not slow:
            for i in range(4):
                setArduinoArm(i, angles[i])

        # Move from init to given angles
        else:
            for i in range(4):
                setArduinoArm(i, int(init_angle[i]))
            ang = np.linspace(init_angle, angles, num=10)
            for a in ang:
                for i in range(4):
                    setArduinoArm(i, int(a[i]))
                    time.sleep(0.001)

        # save it
        self.prev_angle = angles

    def moveByXYZ(self, x, y, z, slow=True, init_angle=None):
        """ Move the arm by given realworld xyz """
        ths = self.xyz2Angle(x, y, z)
        self.moveByAngle(ths, slow=slow, init_angle=init_angle)
        return ths


if __name__ == "__main__":
    from arm_inverse_kinematic import linkTransform, DH
    arm = Arm()
    xyz = [0.16, 0.11, 0.15]
    angle = arm.xyz2Angle(*xyz)
    radangle = angle * np.pi / 180
    T0 =        linkTransform(*DH[0][:3], radangle[0] - np.pi / 2)
    T0[2, 3] += 0.095  # baseline from floor
    T1 = T0.dot(linkTransform(*DH[1][:3], radangle[1]))
    T2 = T1.dot(linkTransform(*DH[2][:3], radangle[2] - np.pi / 2))
    T3 = T2.dot(linkTransform(*DH[3][:3], radangle[3] + np.pi * 3 / 2))
    T4 = T3.dot(linkTransform(*DH[4][:3], 0))

    print("Angle", angle)
    print("Input: ", xyz)
    print("P0", T0[:3, 3])
    print("P1", T1[:3, 3])
    print("P2", T2[:3, 3])
    print("P3", T3[:3, 3])
    print("P4", T4[:3, 3])

    arm.moveByXYZ(*xyz)
