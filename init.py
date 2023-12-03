import numpy as np


def init():

    x_init = np.array([[0],[0],[0]])
    pos_corners = ((0,0),(100,0),(0,100),(100,100))
    trajectory_points = [[1,1],[2,2],[3,3]]

    return x_init, pos_corners, trajectory_points


# If camera has a problem, return x_init = None