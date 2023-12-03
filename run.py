from tdmclient import ClientAsync
import math
import numpy as np
import cv2

from KF import kalman
from motion import motion, motors
from camera import live_cam

    
async def run(x_init, expanded_obs, trajectory_points, cam, corner_coordinates,node,client):
    Tm = 0.2 #5Hz
    estimated_pos = x_init
    delta_distance = 0
    delta_angle = 0
    end = False
    raw_wheel_measurement = np.array([[0],[0]]) #motion(estimated_pos, goal_pos)
    print("run")
    while end == False:
        #raw_wheel_measurement = [[delta_distance],[delta_angle]]
        raw_camera_measurement = live_cam(cam, corner_coordinates, trajectory_points, expanded_obs, x_init)
        #print("raw_wheel_measurement: ", raw_wheel_measurement)
        #print("raw_camera_measurement: ", raw_camera_measurement)
        #print("estimated_pos: ", estimated_pos)
        estimated_pos, P_hat = kalman(raw_wheel_measurement, raw_camera_measurement, estimated_pos)
        print ("estimated pos = ",estimated_pos)
        await node.wait_for_variables({"prox.horizontal"})
        prox_horizontal = node['prox.horizontal']
        left_speed,right_speed,delta_distance,delta_angle,end=motion(estimated_pos,trajectory_points,prox_horizontal) 
        await node.set_variables(motors(left_speed, right_speed))
        await client.sleep(Tm)
        if cv2.waitKey(100) == 27:  # ASCII code for 'Esc' key
            cam.release()
            cv2.destroyAllWindows()
            break    
    await(node.unlock())
    print("#####END#####")
