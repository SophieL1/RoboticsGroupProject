#from tdmclient import ClientAsync
from enum import IntEnum
import math
import numpy as np

#CONSTANTS
SPEED = 100
SPEED_ROT = 50
SPEED_FACTOR =0.51 #0.45  #0.55 # 0.386  
DISTANCE_BETWEEN_WHEELS = 93  #mm
SENSOR_SCALE = 200
MOTOR_SCALE = 20
DISTANCE_TRESHOLD = 5
ROTATION_TRESHOLD = 6
KP = 1.2
#Tm = 0.2 #10 Hz (frequency at which the motors speed change)

class motion_state(IntEnum):
    FOLLOW_TRAJECTORY = 1
    ROTATION = 2

######INTERNAL FUNCTIONS ######

#Convert the thymio speed to mm/s
def conv_thymio_to_mms(thymio_speed):
    speed_mms = thymio_speed*SPEED_FACTOR;
    return speed_mms

def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }

def avoid_obstacle(prox_horizontal):
    Wr = [-40,-40,-40,-20,20,40,20]
    Wl = [20,-20,-40,-40,-40,20,40]
    y = [0,0]
    x = [0,0,0,0,0,0,0]
    for i in range (len(x)):
        x[i]= prox_horizontal[i]//SENSOR_SCALE
        y[0] = y[0]+Wl[i]*x[i]
        y[1] = y[1]+Wr[i]*x[i]
   
    corr_speed_l = y[0]//MOTOR_SCALE
    corr_speed_r = y[1]//MOTOR_SCALE
    return corr_speed_l, corr_speed_r

def p_controller(angle,angle_goal):
    angle_error = angle-angle_goal
    corr_l = - KP*angle_error
    corr_r =  KP*angle_error
    corr_l = int(corr_l)
    corr_r = int(corr_r)
    return corr_l, corr_r

def follow_trajectory(prox_horizontal, distance_goal,angle_goal): #peut être rajouter distance et angle en paramètre
    avoid_speed_l,avoid_speed_r = avoid_obstacle(prox_horizontal)
    #p_corr_l, p_corr_r = p_controller(angle_goal)
    speed_l = SPEED + avoid_speed_l #+ p_corr_l
    speed_r = SPEED + avoid_speed_r #+ p_corr_r
    #print("\t\t angle: ",angle)
    return speed_l,speed_r


# position of the robot between each time intervals (distance in mm and angle in degrees)
def delta_pos(output_speed_l, output_speed_r,time_interval):

    l_speed_mms = conv_thymio_to_mms(output_speed_l)
    r_speed_mms = conv_thymio_to_mms(output_speed_r)
    
    left_distance = l_speed_mms * time_interval
    right_distance = r_speed_mms * time_interval
    delta_distance = (left_distance + right_distance) / 2.0
    
    delta_angle = (-r_speed_mms+l_speed_mms) * time_interval / DISTANCE_BETWEEN_WHEELS
    delta_angle_deg = delta_angle*180/math.pi
    print("\t\t delta_angle_deg : ",delta_angle_deg)
    #print("\t\t delta_distance : ",delta_distance)
    return delta_distance, delta_angle_deg


def relative_pos(pos,pos_goal,idx):
    #pos=[[x],[y],[angle]], pos_goal= [[x1,y1],(x2,y2),...]
    print("pos = ",pos)
    #pos = np.array(pos)
    pos_goal = np.array(pos_goal)
    idx = idx+1
    print("---------------------CALCUL DISTANCE------------------------------")
    print("pos_goal : ", pos_goal)
    print("pos : ", pos)
    print("calcul 1 : ", (pos_goal[idx,0] - pos[0,0]))
    print("calcul 2 : ", (pos_goal[idx,1]- pos_goal[idx,0]))
    relativ_dist = math.sqrt((pos_goal[idx,0] - pos[0,0])**2 + (pos_goal[idx,1]- pos_goal[idx,1])**2)
    print("relative distance : ", relativ_dist)
    print("-------------------------------------------------------------------")
    return relativ_dist, pos[2,0]

def dist_angle_goal(trajectory_points): #[(x1,y1),(x2,y2)]
    dist = []
    angle = []
    for i in range(len(trajectory_points) - 1):
        dx = trajectory_points[i+1][0] - trajectory_points[i][0]
        dy = trajectory_points[i+1][1] - trajectory_points[i][1]

        # Corrected distance calculation
        distance = math.sqrt(dx**2 + dy**2)
        dist.append(distance)

        # Corrected angle calculation
        angle_rad = math.atan2(dy, dx)
        angle_deg = math.degrees(angle_rad)
        angle.append(angle_deg)
    return dist, angle


######PUBLIC FUNCTION####### 
def motion(estimated_pos,pos_goal,prox_horizontal, Tm): # #real parameters:(estimate_pos,pos_goal,distance_goal,angle_goal)
    
    #local variables 
    output_speed_l=0 
    output_speed_r=0
    end = False
    
    #static variables
    if not hasattr(motion, "idx"):
        # If not, initialize idx to 0
        motion.idx = 0
    if not hasattr(motion, "mot_state"):
        # If not, initialize mot_state to the state ROTATION
        motion.mot_state = motion_state.ROTATION
        
    print("\t\t pos_goal: ",pos_goal)
    distance_goal,angle_goal = dist_angle_goal(pos_goal)
        
    if motion.idx<len(distance_goal):
        
        distance,angle = relative_pos(estimated_pos,pos_goal,motion.idx);
        print("\t\t distance_goal: ",distance_goal[motion.idx])
        print("\t\t angle_goal: ",angle_goal[motion.idx])
        print("\t\t distance: ",distance)
        print("\t\t angle: ",angle)
        if motion.mot_state == motion_state.ROTATION:
            angle_rot = angle_goal[motion.idx]-angle
            print("\t\t angle_rot: ",angle_rot)
            if angle_rot<=0: #turn in counterclockwise
                output_speed_l = -(SPEED_ROT) 
                output_speed_r = SPEED_ROT
            else: #turn in clockwise
                output_speed_l = SPEED_ROT
                output_speed_r = -(SPEED_ROT)
            if abs(angle_rot)<ROTATION_TRESHOLD:
                motion.mot_state = motion_state.FOLLOW_TRAJECTORY
                
        # move from a point the following point given by the visibility map   
        if motion.mot_state == motion_state.FOLLOW_TRAJECTORY:
            output_speed_l, output_speed_r = follow_trajectory(prox_horizontal, distance_goal[motion.idx],angle_goal[motion.idx])
            #distance_remaining = distance_goal[motion.idx]-distance
            #if abs(distance_remaining) <= DISTANCE_TRESHOLD:
            if distance <= DISTANCE_TRESHOLD:
                output_speed_l = 0 
                output_speed_r = 0
                print("\t\t distance fin: ",distance)
                motion.mot_state = motion_state.ROTATION
                motion.idx += 1 ### 
                
    else:
        output_speed_l = 0 
        output_speed_r = 0
        end = True
    print("\t\t output speed L : ", output_speed_l)
    print("\t\t output speed R : ", output_speed_r)
    delta_distance, delta_angle = delta_pos(output_speed_l, output_speed_r,Tm)
    return output_speed_l, output_speed_r,delta_distance, delta_angle,end # vrai return: delta_distance, delta_angle,end


#async def move(estimate_pos, node, client):
    #distance_goal = [50, 30]
    #angle_goal = [30, 50]
        
  #  await node.wait_for_variables({"prox.horizontal"})
   # prox_horizontal = node['prox.horizontal']
   # left_speed,right_speed,delta_distance,delta_angle,end=motion(prox_horizontal,distance,angle,distance_goal,angle_goal) 
   # if end == True:
   #     leds = {"leds.top": [0,32,0],}
   #     await node.set_variables(leds)
    
  #  await node.set_variables(motors(left_speed, right_speed))
   # await client.sleep(Tm)
    
    
    
