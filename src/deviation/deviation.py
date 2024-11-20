#!/usr/bin/env python3
import numpy as np
import cv2
import json
from sklearn.neighbors import NearestNeighbors


def calculate_laser_deviation(saved_scan, current_scan, robot_orientation, max_deviation=1.0):
   
    if saved_scan.size == 0 or current_scan.size == 0:
        return 0.0

    # Convert polar to Cartesian coordinates for both scans
    saved_points = polar_to_cartesian(saved_scan)
    current_points = polar_to_cartesian(current_scan)

    # Transform the points to the robot's local frame
    #saved_points_local = transform_to_robot_frame(saved_points, robot_orientation)
    #current_points_local = transform_to_robot_frame(current_points, robot_orientation)

    # Focus on the y-coordinates (lateral deviation in robot's local frame)
    saved_y = saved_points[:, 1]  # y-coordinates from saved scan
    current_y = current_points[:, 1]  # y-coordinates from current scan

    # Find the nearest neighbor indices for the y-coordinate (ignoring x)
    nn = NearestNeighbors(n_neighbors=1)
    nn.fit(saved_y.reshape(-1, 1))  # Fit based on the y-values only
    distances, _ = nn.kneighbors(current_y.reshape(-1, 1))  # Compare y-values only

    # Calculate the average deviation in the y-axis (robot's local frame)
    deviation = np.mean(distances)
    if abs(deviation) > max_deviation:
       # Skip deviations that are too large
        if robot_orientation > 0:
            # Robot is facing right (clockwise), positive deviation means to the right
            return deviation
        else:
            # Robot is facing left (counter-clockwise), positive deviation means to the left
            return -deviation
    else:
        print('discarded')
        return 0.0
    

def polar_to_cartesian(scan):
    """
    Convert polar coordinates (range, angle) to Cartesian coordinates (x, y).
    """
    angles = np.linspace(0, 2 * np.pi, len(scan), endpoint=False)
    x = scan * np.cos(angles)
    y = scan * np.sin(angles)
    points = np.column_stack((x, y))
    return points

def transform_to_robot_frame(points, robot_orientation):
    """
    Transform points from the global frame to the robot's local frame using its orientation.
    
    Args:
    - points: The points to be transformed (Cartesian coordinates).
    - robot_orientation: The robot's yaw orientation (in radians).
    
    Returns:
    - Transformed points in the robot's local frame.
    """
    # Rotation matrix to transform global coordinates to robot's local coordinates
    rotation_matrix = np.array([[np.cos(robot_orientation), -np.sin(robot_orientation)],
                                [np.sin(robot_orientation), np.cos(robot_orientation)]])
    
    # Apply the rotation to each point
    transformed_points = np.dot(points, rotation_matrix.T)  # Rotate points by the robot's orientation
    return transformed_points


def calculate_image_deviation(image1, image2, depth_image, robot_orientation, fx=185.90483944879418, max_match_distance=50.0, max_deviation=1.0):
    """Calculate the deviation distance between two images using ORB and a single depth image."""
    orb = cv2.ORB_create()

    # Convert images to grayscale
    image1_gray = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
    image2_gray = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

    # Detect keypoints and compute descriptors for both images
    kp1, des1 = orb.detectAndCompute(image1_gray, None)
    kp2, des2 = orb.detectAndCompute(image2_gray, None)

    # Match descriptors using the BFMatcher
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
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
            if real_distance_deviation > max_deviation:
                print('discarded')
                continue  # Skip deviations that are too large

            real_distance_deviations.append(real_distance_deviation)

        # Compute the average real-world distance deviation
        if len(real_distance_deviations) > 0:
            angular_deviation = np.mean(real_distance_deviations)
        else:
            angular_deviation = 0.0  # No valid matches found, no deviation

    else:
        angular_deviation = 0.0  # No matches found, no deviation

    # Adjust deviation sign based on the robot's orientation
    if robot_orientation > 0:
        # Robot is facing right (clockwise), positive deviation means right
        return angular_deviation
    else:
        # Robot is facing left (counter-clockwise), positive deviation means left
        return -angular_deviation
