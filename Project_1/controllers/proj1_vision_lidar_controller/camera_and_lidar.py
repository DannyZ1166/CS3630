import math
import numpy as np
from detect_circle import detect_circle

# Instructions:
# Step 1: Review the `detect_circle` function to infer and detect a circle in the image 
#         and compute its angle.
# Step 2: Explore the LiDAR data corresponding to the detected object. Investigate 
#         how to classify the object as either a sphere or a disk. You may reuse 
#         your code from `camera_only.py`.

def camera_and_lidar_calculation(image, camera_fov, object_diameter, lidar_data):
    """
    Performs object detection and classification by combining data from a camera and a LiDAR sensor.

    Args:
        image: The input image captured by the camera.
        camera_fov: The field of view of the camera in radians.
        object_diameter: The expected diameter of the object in meters.
        lidar_data: An array representing LiDAR distance data indexed by angle 
                                  in degrees, where each element corresponds to the distance 
                                  at a specific angle.

    Returns:
        lidar_distance: The distance to the detected object from the LiDAR sensor in meters.
        object_shape: A string indicating the shape of the detected object ("sphere" or "disk").
    """

    ###########################################################################
    # TODO: Student code begins
    ###########################################################################
    
    circles = detect_circle(image)
    if circles is None or len(circles) == 0:
        return None, None  

    largest_circle = max(circles, key=lambda c: c[2])  
    x, y, radius = largest_circle

    object_diameter_pixels = 2 * radius  
    image_width = image.shape[1]
    focal_length = image_width / (2 * np.tan(camera_fov / 2))

    image_center_x = image_width / 2
    angle = (x - image_center_x) * (camera_fov / image_width)

    lidar_angle_index = int(np.degrees(angle))  
    lidar_angle_index = max(0, min(len(lidar_data) - 1, lidar_angle_index))
    lidar_distance = lidar_data[lidar_angle_index]

    angle_range = 8  
    surrounding_distances = lidar_data[max(0, lidar_angle_index - angle_range):min(len(lidar_data), lidar_angle_index + angle_range)]
    lidar_variance = np.var(surrounding_distances)

    if lidar_variance > 0.008:
        object_shape = "sphere"
    else:
        object_shape = "disk"
        
    ###########################################################################
    # Student code ends
    ###########################################################################

    return lidar_distance, object_shape