from tdmclient import ClientAsync

import math
import numpy as np

from KF import kalman
from motion import motion
    
async def run(x_init, goal_pos, pos_init, node, client):
    Tm = 0.2 #5Hz
    estimated_pos = x_init
    delta_distance = 0
    delta_angle = 0
    end = False
    raw_wheel_measurement = np.array([[0],[1]]) #motion(estimated_pos, goal_pos)
    raw_camera_measurement = np.array([[0],[0],[45]]) #camera()
    print("run")
    while end == False:
        raw_wheel_measurement = [[delta_distance],[delta_angle]]
        raw_camera_measurement = np.array([[0],[0],[45]]) #### A changer 
        estimated_pos = kalman(raw_wheel_measurement, raw_camera_measurement, estimated_pos)
        await node.wait_for_variables({"prox.horizontal"})
        prox_horizontal = node['prox.horizontal']
        left_speed,right_speed,delta_distance,delta_angle,end=motion(estimated_pos,goal_pos,prox_horizontal) 
        await node.set_variables(motors(left_speed, right_speed))
        await client.sleep(Tm)
    
    await(node.unlock())
    print("#####END#####")
    
