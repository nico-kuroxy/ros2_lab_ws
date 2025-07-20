#!/usr/bin/python3

"""Coco -> Yolo conversion script.

Author : Nicolas Erbetti 
Mail : nicolas.erbetti.k@gmail.com
Date : 2025/07/19

This file defines the Python script used to convert the coco segmentation dataset obtained with CVAT to a format recognised by yolo.
This step is not required if you export your dataset in a format compatible with segmentation, such as Ultralytics YOLO Segmentation 1.0.
As defined here : https://docs.ultralytics.com/fr/usage/simple-utilities/ 

Copyright (c) Nicolas Erbetti

"""

##################################################################################################
# # DEPENDENCIES
# Python libraries
from ultralytics.data.converter import convert_coco
##################################################################################################



##################################################################################################
# # VARIABLES
# Global variables.
data_directory = "/home/nicolas/personal-website/server/ros2_lab_ws/src/ai/data/online_lab_1"
##################################################################################################



##################################################################################################
# # SCRIPT
# Convert the coco dataset to the yolo one.
convert_coco(labels_dir= data_directory + "/annotations/", save_dir= data_directory + "/coco_converted/", use_segments=True)
##################################################################################################