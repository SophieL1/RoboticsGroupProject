import numpy as np
import math

#### DEFINTIONS ####

### VARIABLE ###
F = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])                             # Define the state transition matrix (F)
#y = np.array([[0], [0], [0]])                                              # Initial output (y = (x,y,alpha))
P = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])                             # Initial prediction (P)
H = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])                             # Define the measurement matrix (H)
Q = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])                             # TODO: Define the process noise covariance (Q)
R = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])                             # TODO: Define the measurement noise covariance (R)


def kalman(raw_wheel_measurement, raw_camera_measurement, estimated_pos):
    x = estimated_pos
    x_hat = x
    u = raw_wheel_measurement
    x_hat, P_hat = kalman_predict(x, u)
    if raw_camera_measurement is not None:
        x_hat, P_hat = kalman_update(x_hat, P_hat, raw_camera_measurement)
    return x_hat


### FUNCTIONS ###
def motion(estimated_pos, goal_pos):
    #TODO: implement motion
    print("motion")
    raw_wheel_measurement = (0,0)
    return raw_wheel_measurement

def camera():
    #TODO: implement motion
    print("camera")
    raw_camera_measurement = (0,0,0)
    return raw_camera_measurement


def B_kalman(x):
    B = np.array([[math.sin(math.radians(x[2])), 0], [math.cos(math.radians(x[2])), 0], [0, 1]])            # Define the control matrix (B)
    return B

def kalman_predict(x, u):
    B = B_kalman(x)
    #print("x: ", x) 
    
    x_hat = F.dot(x) + B.dot(u)
   # print("x_hat: ", x_hat)
    #print("u: ", u)
   # print("F: ", F)
   # print("B: ", B)
   # print("F.dot(x): ", F.dot(x))
   # print("B.dot: ", B.dot)
    P_hat = F.dot(P).dot(F.T) + Q
    return x_hat, P_hat

def kalman_update(x_hat_pred, P_pred, raw_camera_measurement):
    z = raw_camera_measurement

    y = z - H.dot(x_hat_pred)
    S = H.dot(P_pred).dot(H.T) + R
    K = P_pred.dot(H.T).dot(np.linalg.inv(S))
    x_hat_upd = x_hat_pred + K.dot(y)
    P_upd = P_pred - K.dot(H).dot(P_pred)

    return x_hat_upd, P_upd

"""import numpy as np
import math

#### DEFINTIONS ####

### VARIABLE ###
F = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])                             # Define the state transition matrix (F)
#y = np.array([[0], [0], [0]])                                              # Initial output (y = (x,y,alpha))
P = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])                             # Initial prediction (P)
H = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])                             # Define the measurement matrix (H)
Q = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])                             # TODO: Define the process noise covariance (Q)
R = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])                             # TODO: Define the measurement noise covariance (R)


def kalman(raw_wheel_measurement, raw_camera_measurement, estimated_pos):
    x = estimated_pos
    print(x)
    x_hat = x

    u = raw_wheel_measurement
    x_hat, P_hat = kalman_predict(x, u)
    if raw_camera_measurement is not None:
        x_hat = kalman_update(x_hat, P_hat, raw_camera_measurement)
    return x_hat


### FUNCTIONS ###
def motion(estimated_pos, goal_pos):
    #TODO: implement motion
    print("motion")
    raw_wheel_measurement = (0,0)
    return raw_wheel_measurement

def camera():
    #TODO: implement motion
    print("camera")
    raw_camera_measurement = (0,0,0)
    return raw_camera_measurement


def B_kalman(x):
    B = np.array([[math.sin(math.radians(x[2])), 0], [math.cos(math.radians(x[2])), 0], [0, 1]])            # Define the control matrix (B)
    return B

def kalman_predict(x, u):
    B = B_kalman(x)

    x_hat = F.dot(x) + B.dot(u)
    P_hat = F.dot(P).dot(F.T) + Q
    return x_hat, P_hat

def kalman_update(x_hat_pred, P_pred, raw_camera_measurement):
    z = raw_camera_measurement

    y = z - H.dot(x_hat_pred)
    S = H.dot(P_pred).dot(H.T) + R
    K = P_pred.dot(H.T).dot(np.linalg.inv(S))
    x_hat_upd = x_hat_pred + K.dot(y)
    P_upd = P_pred - K.dot(H).dot(P_pred)

    return x_hat_upd, P_upd

"""

