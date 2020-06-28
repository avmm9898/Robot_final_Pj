from win_arduino_connector import setArduinoArm
import numpy as np
import time

pos_init = np.array([90, 90, 90, 90])
pos_targ = np.array([90, 100, 10, 60])
pos = np.linspace(pos_init, pos_targ, num=40)

while True:
    print("init")
    for i in range(4):
        setArduinoArm(i, pos_init[i])
    time.sleep(1)

    print("Run")
    for p in pos:
        for i in range(4):
            setArduinoArm(i, int(p[i]))
    time.sleep(1)

    print("Run")
    for j in reversed(range(len(pos))):
        p = pos[j]
        for i in range(4):
            setArduinoArm(i, int(p[i]))
    time.sleep(1)
