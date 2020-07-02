import numpy as np
import time
from arduino_connector import setArduinoArm
from arm_inverse_kinematic import xyz2angle


def xyz2Armangle(x, y, z):
    """
    The angle for what exactly the arm to move in degree.
    See xyz2angle for more details of x y z from arm base
    """

    permu_th = np.stack([np.linspace(-np.pi / 4, -np.pi / 2, num=10),
                         np.linspace(-np.pi / 4, 0,          num=10)]).T.flatten()

    y -= 0.03
    z += 0.01
    for tht in permu_th:
        angle = xyz2angle(x, y, z, last_angle=tht)
        # The real arm init angle different from baseline
        angle[0] += np.pi / 2
        angle[2] += np.pi / 2
        angle[3] -= np.pi * 3 / 2
        angle = np.mod(angle, 2 * np.pi)

        if np.any(angle > np.pi) or np.any(np.isnan(angle)):
            pass
            # raise ValueError(str(angle) + "is out of working space")
        else:
            # rad to deg
            return np.round(angle * 180 / np.pi).astype(np.int)

    raise ValueError(str(angle) + "is out of working space")


def moveArmByAngle(angles, slow=True, angle_init=[90, 90, 90, 90]):
    if not slow:
        for i in range(4):
            setArduinoArm(i, angles[i])
    else:
        for i in range(4):
            setArduinoArm(i, int(angle_init[i]))
        ang = np.linspace(angle_init, angles, num=10)
        for a in ang:
            for i in range(4):
                setArduinoArm(i, int(a[i]))
                time.sleep(0.001)


def moveArmByXYZ(x, y, z, slow=True, angle_init=[90, 90, 90, 90]):
    """ Give xyz then move arm """
    # get angle for arm
    ths = xyz2Armangle(x, y, z)

    # set arm angle
    moveArmByAngle(ths, slow=slow, angle_init=angle_init)
    return ths


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
