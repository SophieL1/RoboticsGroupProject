import math
import numpy as np
import cv2

from KF import kalman
from camera import live_cam

def run(x_init, expanded_obs, trajectory_points, cam, corner_coordinates):
    estimated_pos = x_init
    #while(1):
    while True:
        raw_camera_measurement = live_cam(cam, corner_coordinates, trajectory_points, expanded_obs, x_init)
        print(raw_camera_measurement)
        raw_wheel_measurement = np.array([[0],[1]]) #motion(estimated_pos, goal_pos)
        #raw_camera_measurement = np.array([[0],[0],[45]]) #camera()
        #estimated_pos = kalman(raw_wheel_measurement, raw_camera_measurement, estimated_pos)
        # Break the loop if the 'Esc' key is pressed
        if cv2.waitKey(100) == 27:  # ASCII code for 'Esc' key
            cam.release()
            cv2.destroyAllWindows()
            break
    

    print("run")
