from arduino_connector import arduino_write_easy
import numpy as np
import time

pos_init = np.array([90, 90, 90, 90])
pos_targ = np.array([90, 100, 10, 60])
pos = np.linspace(pos_init, pos_targ, num=40)

while True:
    print("init")
    for i in range(4):
        arduino_write_easy(f"a{i}", pos_init[i])
    time.sleep(1)

    print("Run")
    for p in pos:
        for i in range(4):
            arduino_write_easy(f"a{i}", int(p[i]))
    time.sleep(1)

    print("Run")
    for j in reversed(range(len(pos))):
        p = pos[j]
        for i in range(4):
            arduino_write_easy(f"a{i}", int(p[i]))
    time.sleep(1)
