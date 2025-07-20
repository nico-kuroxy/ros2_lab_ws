#!/usr/bin/python3

"""Annotation visualization script.

Author : Nicolas Erbetti 
Mail : nicolas.erbetti.k@gmail.com
Date : 2025/07/19

This file defines the Python script used to visualize the annotations from the yolo dataset obtained with the coco2yolo converter before starting the training.
As defined here : https://docs.ultralytics.com/fr/usage/simple-utilities/ 

Copyright (c) Nicolas Erbetti

"""

##################################################################################################
# # DEPENDENCIES
# Python libraries
import cv2
import numpy as np
import os
import time
##################################################################################################



##################################################################################################
# # VARIABLES
# Global variables.
# Path to the directory containing both images and labels.
data_directory = "/home/nicolas/personal-website/server/ros2_lab_ws/src/ai/data/online_lab_2"
# Path to the image directory.
image_dir = data_directory + "/images/default"
# Path to the labels directory.
label_dir = data_directory + "/labels/default"
# Map of labels for the segmentation.
label_map = { 0: "tb3" }
# Rate in hz between two consecutive pictures.
rate = 10
##################################################################################################



##################################################################################################
# # SCRIPT
# Function to visualize the annotation of a single image.
def visualize_annotation(image_path, label_path, label_map):
    # Read the picture with opencv.
    image = cv2.imread(image_path)
    # Check if the picture exists.
    if image is None:
        print(f"Could not read image: {image_path}")
        return
    # Register the picture's dimensions.
    h, w, _ = image.shape
    # Check if the label exists.
    if not os.path.exists(label_path):
        print(f"No annotation file found for {image_path}")
        return
    # Read the labels.
    with open(label_path, 'r') as f:
        lines = f.readlines()
    # Iterate through each lines.
    for line in lines:
        # Split each line to extract the information.
        parts = line.strip().split()
        # Extract the class id as the first information of the line.
        class_id = int(parts[0])
        # Extract the coordinates of the annotated mask as the rest of the information.
        coords = list(map(float, parts[1:]))
        # Convert those coordinates (normalized) to pixel coordinates.
        polygon = np.array([(int(x * w), int(y * h)) for x, y in zip(coords[::2], coords[1::2])], dtype=np.int32)
        # Draw polygon around them.
        cv2.polylines(image, [polygon], isClosed=True, color=(0, 255, 0), thickness=2)
        # Label the class.
        cv2.putText(image, label_map.get(class_id, str(class_id)), tuple(polygon[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Display the image with OpenCV.
    cv2.imshow("YOLOv8 Segmentation Viewer", image)
    # waitKey required to render the images properly.
    cv2.waitKey(1)
    # Wait for 1 second.
    time.sleep(1/rate)

# Get all image files.
image_files = sorted(f for f in os.listdir(image_dir) if f.lower().endswith(('.jpg', '.png', '.jpeg')))

# Show each image.
if not image_files :
    # Error message if no image has been found.
    print("No images found.")
else : 
    # Otherwise, iterate through each image.
    for image_file in image_files:
        img_path = os.path.join(image_dir, image_file)
        lbl_path = os.path.join(label_dir, os.path.splitext(image_file)[0] + ".txt")
        visualize_annotation(img_path, lbl_path, label_map)

# Clean the opencv windows.
cv2.destroyAllWindows()

##################################################################################################