import numpy as np
import time

from arm_inverse_kinematic import xyz2Armangle
from arduino_connector import setArduinoArm


# x: forward
# y: left
# z: up
angles = xyz2Armangle(0.2, 0, 0.1) 
# rad to deg to int
angles = (np.array(angles) / np.pi * 180).astype(np.int)

# fix?
angles[0] += 90

# cut steps of the trajactory
pos_init = np.array([90, 90, 90, 90])
pos = np.linspace(pos_init, angles, num=40).astype(np.int)
# pos = [angles]

# run arm
for p in pos:
    for i in range(4):
        setArduinoArm(i, p[i])
        time.sleep(0.1)
