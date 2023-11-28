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
    
    # Image without the marker on the Thymio
    corners, ids, rejectedImgPoints = aruco.detectMarkers(img, aruco_dict, parameters=parameters)
    for i, marker_id in enumerate(ids):
        #print(corners[i][0])
        x_coordinates = corners[i][0][:, 0]
        y_coordinates = corners[i][0][:, 1]
        min_x, max_x = np.min(x_coordinates)-5, np.max(x_coordinates)+5
        min_y, max_y = np.min(y_coordinates)-5, np.max(y_coordinates)+5
        
        # Create a white square between specified coordinates
        mask = np.zeros_like(img, dtype=np.uint8)
        mask[int(min_y):int(max_y), int(min_x):int(max_x)] = 255
        img_obstacles = cv2.bitwise_or(img, mask)
        
    size_marker = 40
    (h, w) = img_obstacles.shape
    #Images without the markers on the corner and the goal
    mask = np.zeros_like(img_obstacles, dtype=np.uint8)
    mask[0:size_marker, 0:size_marker] = 255
    mask[0:size_marker, w-size_marker:w] = 255
    mask[h-size_marker:h, 0:size_marker] = 255
    mask[h-size_marker:h, w-size_marker:w] = 255
    x1=int(pos_goal[0]-size_marker/2)
    x2=int(pos_goal[0]+size_marker/2)
    y1=int(pos_goal[1]-size_marker/2)
    y2=int(pos_goal[1]+size_marker/2)
    mask[y1:y2, x1:x2] = 255
    img_obstacles = cv2.bitwise_or(img_obstacles, mask)
    
    #reverse the image for the cv2.Canny()
    img_obstacles = cv2.bitwise_not(img_obstacles)
    pos_obstacle = []
    # Apply blur to reduce noise
    blurred = cv2.GaussianBlur(img_obstacles, (9, 9), 0)
    # Apply edge detection
    edges = cv2.Canny(blurred, 150, 200)

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
            #pos = [tuple(point[0]) for point in new_approx]
            pos_obstacle.append(pos)
            
            for point in new_approx:
                cv2.circle(img_obstacles, tuple(point[0]), 5, (0, 255, 0), -1)
    #print("pos_obstacle :", pos_obstacle)
    return img_obstacles,pos_obstacle