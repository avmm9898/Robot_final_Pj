import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import


data = json.load(open("test3.json"))
arm = np.array([i['arm'] for i in data])
vis = np.array([i['axyz'] for i in data])

diff = np.linalg.norm((arm - vis), axis=1)
a = np.sort(diff, axis=0)
arm = arm[diff < a[-2]]
vis = vis[diff < a[-2]]

def show(a, b):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(a[:, 0], a[:, 1], a[:, 2], color='b')
    ax.scatter(b[:, 0], b[:, 1], b[:, 2], color='r')
    for i in range(len(a)):
        ax.plot([a[i, 0], b[i, 0]],
                [a[i, 1], b[i, 1]],
                [a[i, 2], b[i, 2]], 'c-')

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show()


class Quaternion():
    """ Quaternion """
    def __init__(self, c=0, i=0, j=0, k=0, mat=None):
        if mat is not None:
            super().__init__()
            self.mat = mat
            return
        self.mat = np.array([c, i, j, k])

    def __mul__(self, other):
        if type(other) is not Quaternion:
            return NotImplemented
        return Quaternion(mat=self.getRotationMat().dot(other.mat))

    def T(self):
        """ Inverse """
        return Quaternion(self.mat[0], *(-self.mat[1:]))

    def getRotationMat(self):
        """ Output as matrix """
        q = self.mat / np.sqrt(np.sum(self.mat ** 2))
        return np.array([
            [q[0], -q[1], -q[2], -q[3]],
            [q[1],  q[0], -q[3],  q[2]],
            [q[2],  q[3],  q[0], -q[1]],
            [q[3], -q[2],  q[1],  q[0]]])

    def getRotationParam(self):
        """ Rotation based on [x, y, z] with theta """
        q = self.mat / np.linalg.norm(self.mat)
        th = np.arccos(q[0])
        if th < 1e-3:
            return 0, [0, 0, 0]
        # 0 < th < pi
        return th / np.pi * 180 * 2, q[1:] / np.sin(th)

    @classmethod
    def setRotation(cls, th, loc):
        """ Rotation based on [x, y, z] with theta """
        th = th / 180 * np.pi / 2
        return Quaternion(np.cos(th),
                          *(np.sin(th) * np.array(loc)))

    def getIJK(self):
        """ Get i,j,k value """
        return self.mat[1:]


def findTransform(r1, r2):
    """ Find the translation, scaling, rotation of r1 from r2 """
    # normalize
    rr1 = r1 - r1.mean(axis=1)[:, None]
    rr2 = r2 - r2.mean(axis=1)[:, None]

    # find scale
    scale = np.sqrt((rr1 ** 2).sum(axis=0) / (rr2 ** 2).sum(axis=0)).mean()
    rr2 *= scale

    # find rotation
    ## calc convariance matrix
    cov = rr2.dot(rr1.T)
    M = np.array([
        [cov[0,0]+cov[1,1]+cov[2,2], 0, 0, 0],
        [cov[1,2]-cov[2,1], cov[0,0]-cov[1,1]-cov[2,2], 0, 0],
        [cov[2,0]-cov[0,2], cov[0,1]+cov[1,0], cov[1,1]-cov[0,0]-cov[2,2], 0],
        [cov[0,1]-cov[1,0], cov[2,0]+cov[0,2], cov[2,1]+cov[1,2], cov[2,2] - cov[0,0] - cov[1,1]]
        ])
    for i in range(0, 3):
        for j in range(i + 1, 4):
            M[i, j] = M[j, i]

    ## calc rotation matrix
    eigen = np.linalg.eig(M)
    q = Quaternion(mat=eigen[1][:, 0])
    qmat = q.getRotationMat()
    qmat_inv = q.T().getRotationMat()
    qmat_inv[1:4, 1:4] = qmat_inv[1:4, 1:4].T
    R = qmat.dot(qmat_inv)[1:, 1:]

    # find translation
    translate = np.mean(r1 - R.dot(r2.T).T * scale, axis=0)

    # print the parameters
    print("---" * 20)
    print("Eigen", eigen)
    print("translation", translate)
    print("scale", scale)
    print("Rotation", R)
    print("Rotation param", q.getRotationParam())

    s = np.eye(4)
    s[np.arange(3), np.arange(3)] = scale
    g = np.eye(4)
    g[:3, 3] = translate
    g[:3, :3] = R
    transform = g.dot(s)
    print("Transform matrix", transform)

    return transform


r1 = np.hstack([arm, np.ones([len(arm), 1])])
r2 = np.hstack([vis, np.ones([len(vis), 1])])

# gg = findTransform(arm, vis)
# r2 = gg.dot(r2.T).T
a = np.mean(r1 - r2, axis=0)
print(a)
r2 = r2 + a
show(r1, r2)
