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
            cv2.imwrite('0_img.jpg', img)
            # If the image is read successfully, display it
            if img is not None:
                # Conversion to grayscale
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                corner_coordinates = get_corner(gray) #get the position of the corners
                if corner_coordinates is not None: #if we had get the coordinates
                    img_crop = crop_image(gray,corner_coordinates) #crop image
                    cv2.imwrite('1_img_crop.jpg', img_crop)
                    pos_goal = find_goal(img_crop)  #find goal
                    #if pos_goal is not None:
                    if pos_goal is not None and len(pos_goal) == 2:
                        img_obstacles,pos_obstacle = get_obstacles_coordinates(img_crop,pos_goal) #get coordinates of the obstacles
                        pos_thymio = get_position_orientation_thymio(img_crop) #get position of thymio
                        first_time = False
                        print("pos_goal : ", pos_goal)
                        print("pos_obstacle : ",pos_obstacle)
                        print("pos_thymio : ",pos_thymio)
                        robot_dir0 = pos_thymio[2]
                        width = 60
                        width_goal = 50
                        thymio_dimensions = [80, 55, 30, 55]
                        init_pos, expanded_obs, my_coordinates = findPath(pos_obstacle, pos_thymio, robot_dir0, pos_goal, width, width_goal, thymio_dimensions, xlim= 850, ylim=800, verbose=True)
                        init_pos = np.array([[init_pos[0]],[init_pos[1]],[init_pos[2]]])
                        return init_pos, expanded_obs, my_coordinates, cam, corner_coordinates
                
    finally:
        print("")