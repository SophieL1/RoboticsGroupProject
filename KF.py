import numpy as np
import math

#### DEFINTIONS ####

### VARIABLE ###
F = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])                             # Define the state transition matrix (F)
P = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])                             # Initial prediction (P)
H = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])                             # Define the measurement matrix (H)
Q = np.array([[1., 0, 0], [0, 1., 0], [0, 0, 1.]])                             # TODO: Define the process noise covariance (Q)
R = np.array([[0.02, 0, 0], [0, 0.02, 0], [0, 0, 0.2]])                             # TODO: Define the measurement noise covariance (R)


def kalman(raw_wheel_measurement, raw_camera_measurement, estimated_pos, Tm, P_hat):
    x = estimated_pos
    x_hat = x
    u = raw_wheel_measurement
    x_hat, P_hat = kalman_predict(x, u, Tm, P_hat)
    if raw_camera_measurement is not None:
        x_hat, P_hat = kalman_update(x_hat, P_hat, raw_camera_measurement)
    return x_hat, P_hat


### FUNCTIONS ###
def motion(estimated_pos, goal_pos):
    #TODO: implement motion
    raw_wheel_measurement = (0,0)
    return raw_wheel_measurement

def camera():
    #TODO: implement motion
    raw_camera_measurement = (0,0,0)
    return raw_camera_measurement


def B_kalman(x):
    B = np.array([[math.cos(math.radians(x[2])), 0], [math.sin(math.radians(x[2])), 0], [0, 1]])            # Define the control matrix (B)
    return B

def kalman_predict(x, u, Tm, P_hat):
    B = B_kalman(x)
    #print("x: ", x) 
    
    x_hat = F.dot(x) + B.dot(u)
    Q = Q_update(Tm, x_hat)
    P_hat = F.dot(P_hat).dot(F.T) + Q
    #print("P_hat : ", P_hat)
    if x_hat[2][0] > 180:
        x_hat[2][0] = x_hat[2][0] - 360
    if x_hat[2][0] < -180:
        x_hat[2][0] = x_hat[2][0] + 360
    return x_hat, P_hat

def kalman_update(x_hat_pred, P_pred, raw_camera_measurement):
    z = raw_camera_measurement
    y = z - H.dot(x_hat_pred)
    # error not in the range
    if y[2][0] > 180:
        y[2][0] = y[2][0] - 360
    if y[2][0] < -180:
        y[2][0] = y[2][0] + 360
    S = H.dot(P_pred).dot(H.T) + R
    K = P_pred.dot(H.T).dot(np.linalg.inv(S))
    x_hat_upd = x_hat_pred + K.dot(y)
    P_upd = P_pred - K.dot(H).dot(P_pred)

    return x_hat_upd, P_upd

def Q_update(dt, x_hat): #NEW
    speed_variance = 9.515300987244917
    angular_speed_variance = 0.019659712783563876
    Q[0][0] = np.cos(np.radians(x_hat[2][0])) * speed_variance * dt**2
    Q[1][1] = np.sin(np.radians(x_hat[2][0])) * speed_variance * dt**2
    Q[2][2] = angular_speed_variance * dt**2
    return Q

