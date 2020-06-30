import numpy as np


def rotX(th):
    """ Rotation matrix of X """
    c, s = np.cos(th), np.sin(th)
    return np.array([[1, 0,  0],
                     [0, c, -s],
                     [0, s,  c]])


def rotY(th):
    """ Rotation matrix of Y """
    c, s = np.cos(th), np.sin(th)
    return np.array([[ c, 0, s],
                     [ 0, 1, 0],
                     [-s, 0, c]])


def rotZ(th):
    """ Rotation matrix of Z """
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]])


def linkTransform(al, a, d, th):
    """ Transformation matrix """
    ct, st = np.cos(th), np.sin(th)
    ca, sa = np.cos(al), np.sin(al)
    return np.array([[   ct,   -st,   0,     a],
                     [st*ca, ct*ca, -sa, -sa*d],
                     [st*sa, ct*sa,  ca,  ca*d],
                     [    0,     0,   0,     1]])


def getAC():
    """ Custom transformarion from camera to arm base """
    h = 0.15  # z component of origin C
    b = -0.1  # y component of origin C
    th = 0 * -np.pi / 180  # rotation about x axis
    c, s = np.cos(th), np.sin(th)
    return np.array([[1, 0,  0, 0],
                     [0, c, -s, b],
                     [0, s,  c, h],
                     [0, 0,  0, 1]])


# DH table should written in list not variables
DH = [
    # alpha a d theta
    [0      , 0, 0.0625, None],
    [np.pi/2, 0,    0, None],
    [0,   0.115,    0, None],
    [0,   0.105,    0, None],
    [0,   0.127,    0,    0],  # end-effector add here
]


def xyz2angle(x, y, z):
    """
    Read XYZ from camera and output 4 angles on some baseline

    Parameters
    ===========
    X for forwarding
    Y for left
    Z for Upward
    """
    # input position from camera
    # P_TC = np.array([x, y, z, 1])  # position vector of target in carema frame
    # P_AC = getAC().dot(P_TC)  # position vector of target in arm base frame
    P_AC = np.array([x, y, z, 1])

    # redefine it
    al0, a0, d1 = DH[0][:3]
    al1, a1, d2 = DH[1][:3]
    al2, a2, d3 = DH[2][:3]
    al3, a3, d4 = DH[3][:3]
    al4, a4, d5 = DH[4][:3]

    # output position from transformation
    X, Y, Z = P_AC[:3]  # position in arm base frame
    tht = -np.pi / 4  # last joint angle in arm base frame

    R15 = rotX(al1).dot(rotZ(tht))  # orientation of target in frame 1

    P05 = np.array([X, Y, Z, 1])  # position vector of terget in frame 0
    P45_org = [a4, 0, 0]  # position vector of target in frame 4

    th1 = np.arctan2(Y, X)  # theta 1
    T01 = linkTransform(al0, a0, d1, th1)  # transformation form frame 1 to 0

    P15 = np.linalg.inv(T01).dot(P05)  # position vector of origin frame 5 in frame 1
    P15_org = P15[:3]  # position vector of origin frame 5 in frame 1
    P14_org = -R15.dot(P45_org) + P15_org  # position vector of origin frame 4 in frame 1

    th3 = -np.arccos(((np.linalg.norm(P14_org)) ** 2 - (a2 ** 2 + a3 ** 2)) / (2 * a2 * a3))  # th3 from cosine law

    th2 = np.arctan2(P14_org[2], P14_org[0]) - np.arcsin(a3 * np.sin(th3) / np.linalg.norm(P14_org))  # th2 from sine law and arctan

    th4 = -(th2 + th3 - tht)  # th4 from summing of all theta

    return np.array([th1, th2, th3, th4])


if __name__ == "__main__":
    xyz = [0.16, 0.11, 0.15]
    angle = xyz2angle(*xyz)
    T0 =        linkTransform(*DH[0][:3], angle[0])
    T1 = T0.dot(linkTransform(*DH[1][:3], angle[1]))
    T2 = T1.dot(linkTransform(*DH[2][:3], angle[2]))
    T3 = T2.dot(linkTransform(*DH[3][:3], angle[3]))
    T4 = T3.dot(linkTransform(*DH[4][:3], 0))

    print("Angle", angle)
    print("P0", T0[:3, 3])
    print("P1", T1[:3, 3])
    print("P2", T2[:3, 3])
    print("P3", T3[:3, 3])
    print("P4", T4[:3, 3])
    print("Input: ", xyz)
