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