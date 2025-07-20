#!/usr/bin/python3

"""Model training script.

Author : Nicolas Erbetti 
Mail : nicolas.erbetti.k@gmail.com
Date : 2025/07/19

This file defines the Python script used to train the model from the dataset split with the split_dataset script.
Once your model is trained, you can export it to the onxx format to QUANTIZE it and make the inference faster.
You can do it with : "yolo export model=best_s.pt format=onnx dynamic=True"
Ultralytics does not support native quantization during training, so we have to do it afterward... 
The good thing is, the quantized model can be used just like a regular model, if you have the proper version of onnx installed...
Be careful to always have data augmentation in the parameters of your training, to diversify the dataset. 
And also to have "empty pictures" without any class, to teach the model to recognize empty spaces as well and avoid false positive.

Copyright (c) Nicolas Erbetti

"""

##################################################################################################
# # DEPENDENCIES
# Python libraries
from ultralytics import YOLO
import torch
##################################################################################################



##################################################################################################
# # VARIABLES
# Global variables.
# Model from which we start the training. 
# The "s" in the name stands for "small" and represents the size of the model (=the number of its parameters).
# You can use other size, such as "nano" (n). The smaller the model, the faster the inference (but the less precise it becomes!).
base_model = "yolov8n-seg.pt"
##################################################################################################



##################################################################################################
# # SCRIPT
# Look for available GPUs.
print(torch.cuda.is_available())
# Display the name of the defuault gpu, if any.
print(torch.cuda.get_device_name(0))
# Load model architecture (with pre-trained weights for better performances, even though the base weights were never trained to recognize a turtlebot).
model = YOLO(base_model)
# Train the model
results = model.train(
    # Dataset parameter file.
    data="/home/nicolas/personal-website/server/ros2_lab_ws/src/ai/config/yolo.yaml",
    # Training parameters.
    epochs=100, # Number of times the training goes through EVERY picture of the dataset. 
    imgsz=640, # Size to which each training image is resized, Yolo uses squared images as input.
    batch=8, # Number of pictures considered at once during training, ideal for increased training speed.
    project="yolo_training",
    name="seg_from_scratch",
    task="segment",
    # Data augmentation parameters.
    fliplr=0.5,
    flipud=0.2,
    degrees=10,
    translate=0.1,
    scale=0.5,
    shear=2.0,
    perspective=0.001,
    hsv_h=0.015,
    hsv_s=0.7,
    hsv_v=0.4,
    mosaic=True,
    mixup=0.2
)
# Print training results.
print("\n Training from scratch complete.")
print(f"Best model saved at: {results.save_dir}")
print("Final training metrics:")
print(results.results_dict)

##################################################################################################