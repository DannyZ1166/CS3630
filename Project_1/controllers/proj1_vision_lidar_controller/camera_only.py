import numpy as np
import math
from detect_circle import detect_circle


# Instructions:
# Review the `detect_circle` function to infer and detect a circle in the image and compute its angle.

def vision_only_distance_calculation(image, camera_fov, object_diameter):
    """
    This function performs object detection and calculates the distance and angle of the detected circle from a camera sensor.

    Args:
        image: The input image from the camera
        camera_fov: The field of view of the camera in radians.
        object_diameter: The expected diameter of the object in meters.

    Returns:
        distance: The distance to the detected object from camera depth estimation in meters.
        angle: the angle of the detected circle in radians.
    """

    ###########################################################################
    # TODO: Student code begins
    ###########################################################################
    
    circles = detect_circle(image)

    if circles is None or len(circles) == 0:
        return None, None  # No circles detected

    largest_circle = max(circles, key=lambda c: c[2])  
    x, y, radius = largest_circle

    object_diameter_pixels = 2 * radius  
    image_width = image.shape[1]
    focal_length = image_width / (2 * np.tan(camera_fov / 2))

    d = 0.03 + (focal_length * object_diameter) / object_diameter_pixels
    distance = np.sqrt(d**2 + 0.028**2)

    image_center_x = image_width / 2
    angle = (x - image_center_x) * (camera_fov / image_width)
        
    ###########################################################################
    # Student code ends
    ###########################################################################

    return distance, angle