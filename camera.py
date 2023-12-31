import cv2
import numpy as np
from cv2 import aruco

def live_cam(cam, corner_coordinates, my_coordinates, expanded_obs, init_pos, estimated_pos, P):
    """
    visualization of the camera
    - crop image with the 4 corners patterns
    - draw circles and lines on the image
    - get the position/orientation of the Thymio
    """
    #need to change some variables
    init_pos = [init_pos[0][0],init_pos[1][0],init_pos[2][0]]
    estimated_pos = [estimated_pos[0][0],estimated_pos[1][0],estimated_pos[2][0]]
    #read camera
    img = read_camera_image(cam)
    #crop image with the patterns
    img_croped = crop_image(img, corner_coordinates)
    #get Thymio position/orientation
    pos_thymio = get_position_orientation_thymio(img_croped)
    img_croped = visual_img(img_croped, my_coordinates, expanded_obs, init_pos, pos_thymio, estimated_pos, P)
    cv2.imshow('Vision', img_croped)
    if pos_thymio is not None:
        pos_thymio = np.array([[pos_thymio[0]],[pos_thymio[1]],[pos_thymio[2]]])
    return pos_thymio

def visual_img(img_croped, my_coordinates, expanded_obs, init_pos, pos_thymio, estimated_pos, P):
    """
    Draw the best line, add points on obstacles and point on Thymio with
    camera and estimated with the filter
    """
    # draw circle on obstacles
    for coord_list in expanded_obs:
        for coord in coord_list:
            center = (int(coord[0]), int(coord[1]))
            if 0 <= center[0] < 850 and 0 <= center[1] < 800:
                cv2.circle(img_croped, center, radius=5, color=(0, 0, 255), thickness=-1)
    pos1=(int(init_pos[0]),int(init_pos[1]))
    # draw best line
    for coord in my_coordinates:
        pos2=(int(coord[0]), int(coord[1]))
        cv2.line(img_croped, pos1, pos2, (255, 0, 0), 2)
        pos1=pos2
    # draw position of Thymio
    if pos_thymio is not None :
        cv2.circle(img_croped, (int(pos_thymio[0]),int(pos_thymio[1])), radius=10, color=(0, 255, 0), thickness=-1)
    
    x = int(estimated_pos[0])
    y = int(estimated_pos[1])
    alpha = int(estimated_pos[2])

    # Draw the filter and the estimated point
    cv2.ellipse(img_croped, (x, y), (10*abs(int(P[0][0])), 10*abs(int(P[1][1]))), 0, 0, 360, (255, 0, 0), 2)
    cv2.circle(img_croped, (x, y), radius=4, color=(0, 0, 255), thickness=-1)

    return img_croped
    
    
    
def get_corner(gray):
    """
    Get the coordinates of the corners on the real camera image
    """
    height, width = gray.shape
    
    # Initialization of the ArUco dictionary
    aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    
    keypoints_coordinates = []
    
    # Detection of ArUco markers
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # If markers are detected
    if ids is not None:
        for i, marker_id in enumerate(ids):
            if marker_id < 4:  # get only the code 0-1-2-3 which are the corners
                keypoints_coordinates.append(corners[i][0][0])  # Append the corners of the marker
        keypoints_coordinates = [keypoint.astype(int).tolist() for keypoint in keypoints_coordinates]

        # Verify if there are 4 coordinates
        if len(keypoints_coordinates) == 4:
            # reference point for the calculation of the distance
            reference_points = np.array([[0, 0], [width-1, 0], [width-1, height-1], [0, height-1]], dtype=np.float32)

            def distance(point1, point2):
                return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

            # Distance between points and corners
            distances = np.array([[distance(point, ref) for ref in reference_points] for point in keypoints_coordinates])

            # Find corner with min distance for each corner
            closest_references = np.argmin(distances, axis=1)

            # Re-organisation of the points and keep the closest one
            corner_coordinates = [keypoints_coordinates[i] for i in np.argsort(closest_references)]
            print("corner_coordinates OK : ",corner_coordinates)
        else:
            print("Impossible to find the 4 corners !!")
            corner_coordinates = None
    else :
        print("!!! corner_coordinates (2) !!!")
        corner_coordinates = None
    return corner_coordinates;


def find_goal(gray_image):
    """
    Find the coordinate of the goal
    """
    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 10
    params.maxThreshold = 200

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 1000
    params.maxArea = 2000

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.8

    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.8

    # Filter by Inertia3
    params.filterByInertia = True
    params.minInertiaRatio = 0.6
    
    # Create a blob detector with the specified parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs.
    keypoints = detector.detect(gray_image)
    
    pos_goal = []
    # Retrieve the x, y coordinates of each keypoint
    for keypoint in keypoints:
        x, y = keypoint.pt
        
        pos_goal.append(int(x))
        pos_goal.append(int(y))
    if pos_goal == []:
        pos_goal = None
        print("!!! position goal !!!")
    else :
        print("position goal OK: ",pos_goal)
    return pos_goal





def read_camera_image(cam):
    """
    This function read the image from the webcam and return the
    image.
    """
    # Reading the input from the camera
    ret, frame = cam.read()
    
    # If the frame is read successfully, return the frame
    if ret:
        return frame
    else:
        return None
    
    
    
    
    
def crop_image(img, coordinates):
    """
    Crop the image between the 4 patterns in the corners
    """
    new_width = 850
    new_height = 800
    
    # Use the coordinates to obtain the transformation matrix
    original_points = np.array(coordinates, dtype=np.float32)
    destination_points = np.array([[0, 0], [new_width-1, 0], [new_width-1, new_height-1], [0, new_height-1]], dtype=np.float32)

    matrix = cv2.getPerspectiveTransform(original_points, destination_points)

    # Apply the perspective transformation to the original image
    img_crop = cv2.warpPerspective(img, matrix, (new_width, new_height))
    return img_crop




def get_position_orientation_thymio(img):
    """
    Get the position and the orientation of the thymio
    """
    
    # Initialization of the ArUco dictionary
    aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    
    keypoints_coordinates = []
    
    # Detection of ArUco markers
    corners, ids, rejectedImgPoints = aruco.detectMarkers(img, aruco_dict, parameters=parameters)

    # Check if markers are detected and if Thymio marker (ID 4) is present
    if ids is not None and 4 in ids:
        for i, marker_id in enumerate(ids):
            if marker_id == 4:  # get only the 4th which is the thymio

                # Position
                position = np.mean(corners[i][0], axis=0)
                
                # Orientation of the robot
                vector = corners[i][0][0] - corners[i][0][3]
                x_axis_vector = np.array([1, 0])  # Vector along the x-axis
                angle_radians = np.arctan2(np.linalg.det([vector, x_axis_vector]), np.dot(vector, x_axis_vector))
                orientation = -np.degrees(angle_radians)

                keypoints_coordinates.append(corners[i][0][0])  # Append the corners of the marker

        keypoints_coordinates = [keypoint.astype(int).tolist() for keypoint in keypoints_coordinates]

        # Placeholder for position and orientation (modify accordingly)
        raw_camera_measurement = [int(position[0]), int(position[1]), round(orientation, 2)]

        return raw_camera_measurement
    
    # Return None if Thymio marker is not found
    return None



def get_obstacles_coordinates(img,pos_goal):
    """
    Get the corner of the obstacles
        - mask the pattern on the Thymio
        - mask the corner's patterns
        - mask the goal
        - find contours and keep the corners
        - circle the edges on the img
    """
    
    # Initialization of the ArUco dictionary
    aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    
    keypoints_coordinates = []
    
    size_marker = 90
    # Image without the marker on the Thymio
    corners, ids, rejectedImgPoints = aruco.detectMarkers(img, aruco_dict, parameters=parameters)
    for i, marker_id in enumerate(ids):
        x_coordinates = corners[i][0][:, 0]
        y_coordinates = corners[i][0][:, 1]
        min_x, max_x = np.min(x_coordinates)-size_marker, np.max(x_coordinates)+size_marker
        min_y, max_y = np.min(y_coordinates)-size_marker, np.max(y_coordinates)+size_marker
        print("-------------------", min_x, max_x, min_y, max_y)
        # Create a white square between specified coordinates
        mask = np.zeros_like(img, dtype=np.uint8)
        mask[int(min_y):int(max_y), int(min_x):int(max_x)] = 255
        img_obstacles = cv2.bitwise_or(img, mask)
        
    size_marker = 70
    (h, w) = img_obstacles.shape
    #Images without the markers on the corner and the goal
    mask = np.zeros_like(img_obstacles, dtype=np.uint8)
    mask[0:size_marker, 0:size_marker] = 255
    mask[0:size_marker, w-size_marker:w] = 255
    mask[h-size_marker:h, 0:size_marker] = 255
    mask[h-size_marker:h, w-size_marker:w] = 255
    x1=int(pos_goal[0]-size_marker)
    x2=int(pos_goal[0]+size_marker)
    y1=int(pos_goal[1]-size_marker)
    y2=int(pos_goal[1]+size_marker)
    mask[y1:y2, x1:x2] = 255
    img_obstacles = cv2.bitwise_or(img_obstacles, mask)
    cv2.imwrite('2_img_obstacles.jpg', img_obstacles)
    
    #reverse the image for the cv2.Canny()
    img_obstacles = cv2.bitwise_not(img_obstacles)
    pos_obstacle = []
    # Apply blur to reduce noise
    blurred = cv2.GaussianBlur(img_obstacles, (9, 9), 0)
    # Apply edge detection
    edges = cv2.Canny(blurred, 150, 200)
    cv2.imwrite('3_img_contours.jpg', img_obstacles)
    # Find contours in the image
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Loop through the contours
    for contour in contours:
        # Approximate the contour with a polygon
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        # Merge points that are too close        
        def distance(p1, p2):
            return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

        new_approx = []
        merged_points = set()

        for i in range(len(approx)):
            if i not in merged_points:
                current_point = approx[i][0]

                # Find points that are too close
                close_points = [j for j in range(i + 1, len(approx)) if distance(current_point, approx[j][0]) < 10]

                # Calculate the average of close points and update the list
                if close_points:
                    merged_points.update(close_points)
                    close_points.append(i)
                    average_point = np.mean(np.array([approx[j][0] for j in close_points]), axis=0)
                    new_approx.append([average_point.astype(int)])
                else:
                    new_approx.append([current_point])
        if len(new_approx)>2 and len(new_approx)<7:
            # Convert the resulting list to a NumPy array
            new_approx = np.array(new_approx)
            pos = [list(point[0]) for point in new_approx]
            pos_obstacle.append(pos)
            
            for point in new_approx:
                cv2.circle(img_obstacles, tuple(point[0]), 5, (0, 255, 0), -1)
    return img_obstacles,pos_obstacle