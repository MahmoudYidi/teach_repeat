#!/usr/bin/env python3
import numpy as np
import cv2
import json
from sklearn.neighbors import NearestNeighbors


def calculate_laser_deviation(saved_scan, current_scan, robot_orientation, max_deviation=0.8):
   
    if saved_scan.size == 0 or current_scan.size == 0:
        return 0.0

    # Convert polar to Cartesian coordinates for both scans
    saved_points = polar_to_cartesian(saved_scan)
    current_points = polar_to_cartesian(current_scan)

    #  y-coordinates (lateral deviation in robot's local frame)
    saved_y = saved_points[:, 1]  
    current_y = current_points[:, 1]  

    # Find the nearest neighbor indices 
    nn = NearestNeighbors(n_neighbors=1)
    nn.fit(saved_y.reshape(-1, 1)) 
    distances, _ = nn.kneighbors(current_y.reshape(-1, 1))  

    # Calculate the average deviation in the y-axis (robot's local frame)
    deviation = np.mean(distances)
    if abs(deviation) < max_deviation:
            return deviation    
            
    else:
        #print('discarded')
        return 0.0
    

def polar_to_cartesian(scan):
   #Convert polar coordinates (range, angle) to Cartesian coordinates (x, y).
    angles = np.linspace(0, 2 * np.pi, len(scan), endpoint=False)
    x = scan * np.cos(angles)
    y = scan * np.sin(angles)
    points = np.column_stack((x, y))
    return points



def calculate_image_deviation(image1, image2, depth_image, robot_orientation, fx=185.90483944879418, 
                              max_match_distance=200.0, max_deviation=0.8, use_sift=False):
    """
    Calculate the deviation distance(m) between two images using ORB 

    Parameters:
    - image1, image2: Input RGB images to compare.
    - depth_image: Depth image corresponding to `image1`.
    - robot_orientation: Robot's orientation for deviation adjustment.
    - fx: Camera's focal length in pixels.
    - max_match_distance: Maximum distance for valid feature matches.
    - max_deviation: Maximum allowed real-world deviation.
    - use_sift: Use SIFT if True; otherwise, use ORB.

    Returns:
    - angular_deviation: Calculated angular deviation.
    """
    # Choose the feature detector
    if use_sift:
        feature_detector = cv2.SIFT_create()
        norm_type = cv2.NORM_L2
    else:
        feature_detector = cv2.ORB_create()
        norm_type = cv2.NORM_HAMMING

    # Convert images to grayscale
    image1_gray = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
    image2_gray = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

    # Detect keypoints and compute descriptors for both images
    kp1, des1 = feature_detector.detectAndCompute(image1_gray, None)
    kp2, des2 = feature_detector.detectAndCompute(image2_gray, None)

    # Match descriptors using the BFMatcher
    bf = cv2.BFMatcher(norm_type, crossCheck=True)
    matches = bf.match(des1, des2)
    
    # Sort matches by their distance
    matches = sorted(matches, key=lambda x: x.distance)

    if len(matches) > 0:
        # List to hold real-world horizontal distance deviations
        real_distance_deviations = []

        for match in matches:
            # Only consider matches that are within the specified threshold
            if match.distance > max_match_distance:
                continue  # Skip matches that are too far apart

            # Get the keypoint locations for the current match
            pt1 = kp1[match.queryIdx].pt  # (x, y) in image1
            pt2 = kp2[match.trainIdx].pt  # (x, y) in image2

            # Get the corresponding depth value from the depth image for the first image
            depth1 = depth_image[int(pt1[1]), int(pt1[0])]  # Depth for keypoint in image1

            # Calculate horizontal offset (in pixels) between the two keypoints
            horizontal_offset = pt2[0] - pt1[0]

            # If the depth value is valid, calculate the real-world horizontal distance
            real_distance_deviation = (horizontal_offset * depth1) / fx
            if abs(real_distance_deviation) > max_deviation:
                continue  # Skip deviations that are too large

            real_distance_deviations.append(real_distance_deviation)

        # Compute the average real-world distance deviation
        if len(real_distance_deviations) > 0:
           return np.mean(real_distance_deviations)
        else:
           return  0.0  # No valid matches found, no deviation

    else:
        return  0.0  # No matches found, no deviation

