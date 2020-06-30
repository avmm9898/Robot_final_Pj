import numpy as np
import time
from arduino_connector import setArduinoArm
from arm_inverse_kinematic import xyz2angle

pos_init = np.array([90, 90, 90, 90])


def xyz2Armangle(x, y, z):
    """
    The angle for what exactly the arm to move in degree.
    See xyz2angle for more details of x y z input
    """
    angle = xyz2angle(x, y, z)

    # The real arm init angle different from baseline
    angle[0] += np.pi / 2
    angle[2] += np.pi / 2
    angle[3] -= np.pi * 3 / 2
    angle = np.mod(angle, 2 * np.pi)

    if np.any(angle < 0) or np.any(angle > np.pi):
        print(angle)
        raise ValueError("Out of working space")

    # rad to deg
    return np.round(angle * 180 / np.pi).astype(np.int)


def moveArmByXYZ(x, y, z, slow=False):
    """ Give xyz then move arm """
    # get angle for arm
    ths = xyz2Armangle(*xyz)

    # set arm angle
    if not slow:
        for i in range(4):
            setArduinoArm(i, ths[i])
    else:
        for i in range(4):
            setArduinoArm(i, int(pos_init[i]))
        pos_targ = np.array(ths)
        pos = np.linspace(pos_init, pos_targ, num=40)
        for p in pos:
            for i in range(4):
                setArduinoArm(i, int(p[i]))
                time.sleep(0.001)


if __name__ == "__main__":
    from arm_inverse_kinematic import linkTransform, DH
    xyz = [0.16, 0.11, 0.15]
    angle = xyz2Armangle(*xyz)
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

    moveArmByXYZ(*xyz, slow=True)
