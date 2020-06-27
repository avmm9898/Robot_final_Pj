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
    h = 0.1  # z component of origin C
    b = -0.1  # y component of origin C
    th = 0 * -np.pi / 180  # rotation about x axis
    c, s = np.cos(th), np.sin(th)
    return np.array([[1, 0,  0, 0],
                     [0, c, -s, b],
                     [0, s,  c, h],
                     [0, 0,  0, 1]])


# DH table should written in list not variables
DH_param = []
al0 = 0
al1 = np.pi/2
al2 = 0
al3 = 0
al4 = 0

a0 = 0
a1 = 0
a2 = 0.12
a3 = 0.105
a4 = 0.0494

d1 = 0.05
d2 = 0
d3 = 0
d4 = 0
d5 = 0


def xyz2Armangle(x, y, z):
    """ Read XYZ from camera and output 4 arm angle """
    # input position from camera
    P_TC = np.array([x, y, z, 1])  # position vector of target in carema frame
    P_AC = getAC().dot(P_TC)  # position vector of target in arm base frame

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

    return [th1, th2, th3, th4]


if __name__ == "__main__":
    xyz = [0.1, 0., 0.05]
    theta = xyz2Armangle(*xyz)
    print("Theta", theta)
    T0e = linkTransform(al0, a0, d1, theta[0]).dot(
          linkTransform(al1, a1, d2, theta[1])).dot(
          linkTransform(al2, a2, d3, theta[2])).dot(
          linkTransform(al3, a3, d4, theta[3])).dot(
          linkTransform(al4, a4,  0,  0))
    print("Input: ", xyz)
    print("Output:", np.linalg.inv(getAC()).dot(T0e)[:, 3])
