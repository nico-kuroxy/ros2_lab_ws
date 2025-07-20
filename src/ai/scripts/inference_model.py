#!/usr/bin/python3

"""Model inference script.

Author : Nicolas Erbetti 
Mail : nicolas.erbetti.k@gmail.com
Date : 2025/07/19

This file defines the Python script used to run the inference on the pytorch model trained with train_model.py.

Copyright (c) Nicolas Erbetti

"""

##################################################################################################
# # DEPENDENCIES
# Python libraries
from ultralytics import YOLO
import glob
import os
##################################################################################################



##################################################################################################
# # VARIABLES
# Global variables.
# Path to the directory containing both images and labels.
data_directory = "/home/nicolas/personal-website/server/ros2_lab_ws/src/ai/data/online_lab_1/coco_converted"
# Path to the image directory.
image_dir = data_directory + "/images/default"
# Path to the output directory, where the segmented images will be saved.
output_dir = data_directory + "/images/output"
# Path to the trained model.
model_path = "yolo_training/seg_from_scratch3/weights/best.pt"
# Confidence threshold for segmentation.
confidence_threshold = 0.25
##################################################################################################



##################################################################################################
# # SCRIPT
# Load model.
model = YOLO(model_path)
# Check the existence of the output folder.
os.makedirs(output_dir, exist_ok=True)
# Retrieve each image of the input folder.
image_paths = glob.glob(f"{image_dir}/*.jpg") + glob.glob(f"{image_dir}/*.png")
# Run inference on each image.
for image_path in image_paths:
    # Log the process.
    print(f"Processing: {image_path}")
    # Predict the results.
    results = model.predict(
        source=image_path,
        conf=confidence_threshold,
        save=True,
        save_txt=True,
        save_crop=False,
        project=output_dir,
        name="",
        exist_ok=True
    )
# Log the end of the inference.
print(f"\n Annotated results saved in: {output_dir}")

##################################################################################################