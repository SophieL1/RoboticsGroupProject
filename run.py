from tdmclient import ClientAsync
import math
import numpy as np
import cv2
import time

from KF import kalman
from motion import motion, motors
from camera import live_cam


    
    
async def run(x_init, expanded_obs, trajectory_points, cam, corner_coordinates,node,client):
    #Tm = 0.2 #5Hz
    estimated_pos = x_init
    delta_distance = 0
    delta_angle = 0
    end = False
    raw_wheel_measurement = np.array([[0],[0]]) #motion(estimated_pos, goal_pos)
    old_time = None
    x_pos_init = [x_init[0][0],x_init[1][0]]
    trajectory_points.insert(0, x_pos_init)
    P = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) 
    while end == False:
        raw_camera_measurement = live_cam(cam, corner_coordinates, trajectory_points, expanded_obs, x_init, estimated_pos, P)
        actual_time = time.time()
        if old_time is not None :
            Tm = actual_time - old_time
        else :
            Tm = 0
        estimated_pos, P = kalman(raw_wheel_measurement, raw_camera_measurement, estimated_pos, Tm, P)
        await node.wait_for_variables({"prox.horizontal"})
        prox_horizontal = node['prox.horizontal']
        left_speed,right_speed,delta_distance,delta_angle,end=motion(estimated_pos,trajectory_points,prox_horizontal,Tm) 
        raw_wheel_measurement = np.array([[delta_distance],[delta_angle]])
        await node.set_variables(motors(left_speed, right_speed))
        old_time = actual_time
        if cv2.waitKey(100) == 27:  # ASCII code for 'Esc' key
            cam.release()
            cv2.destroyAllWindows()
            break    
    await(node.unlock())
    print("#####END#####")