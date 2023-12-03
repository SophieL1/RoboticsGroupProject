import cv2
from cv2 import aruco
import numpy as np
from matplotlib import pyplot as plt
import time

#fonction Sophie
from findPath import findPath

#fonction Liandro
from camera import read_camera_image, get_corner, find_goal, crop_image, get_position_orientation_thymio, get_obstacles_coordinates

def init():
    cam_port = 1 #camera 1 in my computer
    cam = cv2.VideoCapture(cam_port)
    first_time = True

    # Check if the camera opened successfully
    if not cam.isOpened():
        print("Error opening camera")
        return None

    try:
        while True:
            if first_time: # wait for focus of the camera because firsts image are not good
                img = read_camera_image(cam)
                time.sleep(0.5)
                first_time = False
                
            # Read the camera image
            img = read_camera_image(cam)

            # If the image is read successfully, display it
            if img is not None:
                # Conversion to grayscale
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                
                corner_coordinates = get_corner(gray) #get the position of the corners
                if corner_coordinates is not None: #if we had get the coordinates
                    img_crop = crop_image(gray,corner_coordinates) #crop image
                    pos_goal = find_goal(img_crop)  #find goal
                    if pos_goal is not None:
                        img_obstacles,pos_obstacle = get_obstacles_coordinates(img_crop,pos_goal) #get coordinates of the obstacles
                        pos_thymio = get_position_orientation_thymio(img_crop) #get position of thymio
                        first_time = False
                        print("pos_goal : ", pos_goal)
                        print("pos_obstacle : ",pos_obstacle)
                        print("pos_thymio : ",pos_thymio)
                        """width=60
                        goal_pos = [[pos_goal[0]-20, pos_goal[1]-20], [pos_goal[0]-20, pos_goal[1]+20], [pos_goal[0]+20, pos_goal[1]+20], [pos_goal[0]+20, pos_goal[1]-20]]
                        start_pos = [[pos_thymio[0]-50, pos_thymio[1]-50], [pos_thymio[0]-50, pos_thymio[1]+50], [pos_thymio[0]+50, pos_thymio[1]+50], [pos_thymio[0]+50, pos_thymio[1]-50]]
                        
                        #width = 0.2
                        #obs_list = [[[300, 200], [400, 200], [400, 300], [300, 300]], [[100, 200], [200, 400], [100, 400]]]
                        #start_pos = [[100, 100], [150, 100], [150, 150], [100, 150]]
                        robot_dir0 = 0
                        #goal_pos = [[450, 450], [500, 450], [500, 500], [450, 500]]
                        findPath(pos_obstacle, start_pos, robot_dir0, goal_pos, width)
                        #findPath(pos_obstacle, start_pos, pos_thymio[2], pos_goal, width)"""
                        robot_dir0 = pos_thymio[2]
                        #goal_pos = [[450, 450], [500, 450], [500, 500], [450, 500]]
                        width = 60
                        width_goal = 50
                        thymio_dimensions = [80, 55, 30, 55]
                        init_pos, expanded_obs, my_coordinates = findPath(pos_obstacle, pos_thymio, robot_dir0, pos_goal, width, width_goal, thymio_dimensions, xlim= 850, ylim=800, verbose=True)
                        
                        return init_pos, expanded_obs, my_coordinates, cam, corner_coordinates
                
    finally:
        print("")
        # Release the camera capture object and close all windows
        #cam.release()
        #cv2.destroyAllWindows()
        #return None