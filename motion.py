#from tdmclient import ClientAsync
from enum import IntEnum
import math
import numpy as np

#CONSTANTS
SPEED = 100
SPEED_ROT = 50
SPEED_FACTOR = 0.378 
DISTANCE_BETWEEN_WHEELS = 93  #mm
SENSOR_SCALE = 200
MOTOR_SCALE = 20
DISTANCE_TRESHOLD = 20
ROTATION_TRESHOLD = 6
KP = 1 #1.2

#ANN CONSTANTS
WR = [-80,-80,-80,-40,60,0,0] 
WL = [60,-40,-80,-80,-80,0,0] 

class motion_state(IntEnum):
    FOLLOW_TRAJECTORY = 1
    ROTATION = 2


######INTERNAL FUNCTIONS ######

#Convert the thymio speed to mm/s
def conv_thymio_to_mms(thymio_speed):
    speed_mms = thymio_speed*SPEED_FACTOR
    return speed_mms

def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }

#Artificial neural network between the proximity sensors and the motors
def avoid_obstacle(prox_horizontal):
    y = [0,0]
    x = [0,0,0,0,0,0,0]
    for i in range (len(x)):
        x[i]= prox_horizontal[i]//SENSOR_SCALE
        y[0] = y[0]+WL[i]*x[i]
        y[1] = y[1]+WR[i]*x[i]
   
    corr_speed_l = y[0]//MOTOR_SCALE
    corr_speed_r = y[1]//MOTOR_SCALE
    return corr_speed_l, corr_speed_r

def p_controller(angle,angle_goal):
    angle_error = angle-angle_goal
    if angle_error > 180:
        angle_error = angle_error - 360
    if angle_error < -180:
        angle_error = angle_error + 360
    corr_l = - KP*angle_error
    corr_r =  KP*angle_error
    corr_l = int(corr_l)
    corr_r = int(corr_r)
    return corr_l, corr_r

# Correction of the motors speed
def motors_correction(prox_horizontal,angle_goal,angle): 
    avoid_speed_l,avoid_speed_r = avoid_obstacle(prox_horizontal)
    p_corr_l, p_corr_r = p_controller(angle,angle_goal)
    speed_l = SPEED + avoid_speed_l + p_corr_l
    speed_r = SPEED + avoid_speed_r + p_corr_r
    return speed_l,speed_r


# Position of the robot between each time intervals (distance in mm and angle in degrees)
def delta_pos(output_speed_l, output_speed_r,time_interval):

    l_speed_mms = conv_thymio_to_mms(output_speed_l)
    r_speed_mms = conv_thymio_to_mms(output_speed_r)
    
    left_distance = l_speed_mms * time_interval
    right_distance = r_speed_mms * time_interval
    delta_distance = (left_distance + right_distance) / 2.0
    
    delta_angle = (-r_speed_mms+l_speed_mms) * time_interval / DISTANCE_BETWEEN_WHEELS
    delta_angle_deg = delta_angle*180/math.pi
    return delta_distance, delta_angle_deg

#Compute the distance and angle between the thymio position and the next trajectory point/position goal
def relative_pos(pos,pos_goal,idx): #pos=[[x],[y],[angle]], pos_goal= [[x1,y1],(x2,y2),...]
    pos_goal = np.array(pos_goal)
    idx = idx+1
    relativ_dist = math.sqrt((pos_goal[idx,0] - pos[0,0])**2 + (pos_goal[idx,1]- pos[1,0])**2)
    return relativ_dist, pos[2,0]

#Compute a list with the angle between each consecutive trajectory points (angle) and 
#the angle between the robot position and the next trajectory point (angle_goal_live)
def calcul_angle(trajectory_points, estimated_pos, index): #[(x1,y1),(x2,y2)]
    angle = []
    for i in range(len(trajectory_points) - 1):
        dx = trajectory_points[i+1][0] - trajectory_points[i][0]
        dy = trajectory_points[i+1][1] - trajectory_points[i][1]

        # Corrected angle calculation
        angle_rad = math.atan2(dy, dx)
        angle_deg = math.degrees(angle_rad)
        angle.append(angle_deg)
    
    dx = trajectory_points[index+1][0] - estimated_pos[0]
    dy = trajectory_points[index+1][1] - estimated_pos[1]
    angle_goal_live = math.degrees(math.atan2(dy, dx))
    return angle,angle_goal_live

def normalize_angle(angle):
    normalized_angle = angle % 360.0
    if normalized_angle > 180.0:
        normalized_angle -= 360.0
    return normalized_angle


######PUBLIC FUNCTION####### 

def motion(estimated_pos,pos_goal,prox_horizontal, Tm): 
    
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
        
    angle_goal,angle_goal_live = calcul_angle(pos_goal, estimated_pos, motion.idx)
        
    if motion.idx<len(angle_goal):
        distance,angle = relative_pos(estimated_pos,pos_goal,motion.idx)
        angle = normalize_angle(angle)

        #Rotate until the robot reach the direction of the next trajectory point/position goal
        if motion.mot_state == motion_state.ROTATION:
            angle_rot = angle_goal[motion.idx]-angle
            if angle_rot > 180:
                angle_rot = angle_rot-360
            elif angle_rot < -180:
                angle_rot = angle_rot+360
            if angle_rot<=0: #turn in counterclockwise
                output_speed_l = -(SPEED_ROT) 
                output_speed_r = SPEED_ROT
            else: #turn in clockwise
                output_speed_l = SPEED_ROT
                output_speed_r = -(SPEED_ROT)
            if abs(angle_rot)<ROTATION_TRESHOLD:
                motion.mot_state = motion_state.FOLLOW_TRAJECTORY

        #Move from a point the following point given by the visibility map   
        if motion.mot_state == motion_state.FOLLOW_TRAJECTORY:
            output_speed_l, output_speed_r = motors_correction(prox_horizontal, angle_goal_live, angle)
            if distance <= DISTANCE_TRESHOLD:
                output_speed_l = 0 
                output_speed_r = 0
                motion.mot_state = motion_state.ROTATION
                motion.idx += 1  
                
    else:
        output_speed_l = 0 
        output_speed_r = 0
        end = True

    delta_distance, delta_angle = delta_pos(output_speed_l, output_speed_r,Tm)
    
    return output_speed_l, output_speed_r,delta_distance, delta_angle,end 
    
    
    
