import numpy as np
import matplotlib.pyplot as plt




# transformarion form camere to arm base
def T_AC ():
    h = 0.1 # z component of origin C 
    b = -0.1 # y component of origin C 
    th = -np.pi*0/180 # rotation about x axis
    c = np.cos(th)
    s = np.sin(th)
    T_AC = np.array ([[1,0,0,0],[0,c,-s,b],[0,s,c,h],[0,0,0,1]]) # transformation matrix
    return T_AC

# rotation matrix of X
def RotX(th): 
    c = np.cos(th) # define trigonomertic function
    s = np.sin(th) # define trigonomertic function
    RotX = np.array ([[1,0,0],[0,c,-s],[0,s,c]])
    return RotX

# rotation matrix of y
def RotY(th): 
    c = np.cos(th) # define trigonomertic function
    s = np.sin(th) # define trigonomertic function
    RotY = np.array ([[c,0,s],[0,1,0],[-s,0,c]])
    return RotY

# rotation matrix of z
def RotZ(th): 
    c = np.cos(th) # define trigonomertic function
    s = np.sin(th) # define trigonomertic function
    RotZ = np.array ([[c,-s,0],[s,c,0],[0,0,1]]) 
    return RotZ

# transformation matrix 
def Get_Transformation(al,a,d,th):
    ct = np.cos(th)
    st = np.sin(th)
    ca = np.cos(al)
    sa = np.sin(al)
    T = np.array ([[ct,-st,0,a],[st*ca,ct*ca,-sa,-sa*d],[st*sa,ct*sa,ca,ca*d],[0,0,0,1]])
    return T

def Img2ArmAngle(x, y, z):
    # input position from camera
    x = 0.1
    y = 0.
    z = 0.05
    P_TC = np.array ([x,y,z,1]) # position vector of target in carema frame

    P_AC = T_AC().dot(P_TC) # position vector of target in arm base frame

    # print(P_AC)

    # DH table
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

    th1 = 0
    th2 = np.pi*3/4
    th3 = -np.pi/2
    th4 = -np.pi/2
    th5 = 0

    # output position from transformation
    X = P_AC[0] # x component position in arm base frame
    Y = P_AC[1] # y component position in arm base frame
    Z = P_AC[2] # z component position in arm base frame
    tht = -np.pi/4 # last joint angle in arm base frame

    R15 = RotX(al1).dot(RotZ(tht)) # orientation of target in frame 1

    P05 = np.array ([X,Y,Z,1]) # position vector of terget in frame 0
    P45_org = [a4,0,0] # position vector of target in frame 4

    th1 = np.arctan2(Y,X) # theta 1

    T01 = Get_Transformation(al0,a0,d1,th1) # transformation form frame 1 to 0

    P15 = (np.linalg.inv(T01)).dot(P05) # position vector of origin frame 5 in frame 1
    P15_org = ([[P15[0],P15[1],P15[2]]]) #  position vector of origin frame 5 in frame 1
    P14_org = -R15.dot(P45_org) + P15_org # position vector of origin frame 4 in frame 1

    th3 = -np.arccos(((np.linalg.norm(P14_org))**2-(a2**2+a3**2))/(2*a2*a3)) # th3 from cosine law

    th2 = np.arctan2(P14_org[0,2],P14_org[0,0]) - np.arcsin(a3*np.sin(th3)/np.linalg.norm(P14_org)) # th2 from sine law and arctan

    th4 = -(th2+th3-tht) # th4 from summing of all theta

    theta=[th1,th2,th3,th4]
    print (th1,th2,th3,th4)

    return theta
