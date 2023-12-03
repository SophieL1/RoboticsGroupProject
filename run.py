import math
import numpy as np

from KF import kalman

def run(x_init, goal_pos, pos_init):
    estimated_pos = x_init
    #while(1):
    raw_wheel_measurement = np.array([[0],[1]]) #motion(estimated_pos, goal_pos)
    raw_camera_measurement = np.array([[0],[0],[45]]) #camera()
    estimated_pos = kalman(raw_wheel_measurement, raw_camera_measurement, estimated_pos)

    print("run")
