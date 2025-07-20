#!/usr/bin/python3

"""Split training dataset.

Author : Nicolas Erbetti 
Mail : nicolas.erbetti.k@gmail.com
Date : 2025/07/19

This file defines the Python script used to split the images/labels in the default folder into the "train", "validate" and "test" datasets.

Copyright (c) Nicolas Erbetti

"""


##################################################################################################
# # DEPENDENCIES
# Python libraries
import os
import random
import shutil
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
# Train ratio.
train_ratio = 0.7
# Validate ratio.
validate_ratio = 0.2
# Test ratio.
test_ratio = 0.1
##################################################################################################



##################################################################################################
# # SCRIPT
# Register output split directories.
splits = ["train", "validate", "test"]
image_dirs = {s: os.path.join(data_directory, "images", s) for s in splits}
label_dirs = {s: os.path.join(data_directory, "labels", s) for s in splits}
# Create split folders.
for dir_path in list(image_dirs.values()) + list(label_dirs.values()):
    os.makedirs(dir_path, exist_ok=True)
# Get all image files.
image_files = [f for f in os.listdir(image_dir) if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
# Shuffle the images.
random.shuffle(image_files)
# Compute split sizes.
total = len(image_files)
train_end = int(total * train_ratio)
val_end = train_end + int(total * validate_ratio)
# Assign each image to its folder.
train_files = image_files[:train_end]
val_files = image_files[train_end:val_end]
test_files = image_files[val_end:]
# Create a split map.
split_map = {
    "train": train_files,
    "validate": val_files,
    "test": test_files
}
# Move files to split folders.
for split, files in split_map.items():
    for img_file in files:
        # Find the corresponding label.
        label_file = os.path.splitext(img_file)[0] + ".txt"
        # Retrieve the image path from the original folder.
        src_img = os.path.join(image_dir, img_file)
        # Retrieve the label path from the original folder.
        src_lbl = os.path.join(label_dir, label_file)
        # Set the image path in the destination folder.
        dst_img = os.path.join(image_dirs[split], img_file)
        # Set the label path in the destination folder.
        dst_lbl = os.path.join(label_dirs[split], label_file)
        # Move the image if it exists.
        if os.path.exists(src_img):
            shutil.move(src_img, dst_img)
        # Move the label if it exists.
        if os.path.exists(src_lbl):
            shutil.move(src_lbl, dst_lbl)
    # Log the split of each folder.
    print(f"{split.title()}: {len(files)} images")
# Log the end of the splitting.
print("Dataset split complete.")

##################################################################################################