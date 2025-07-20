#!/usr/bin/env python3

"""Yolo segmentation ros2 node in Python.

Author : Nicolas Erbetti 
Mail : nicolas.erbetti.k@gmail.com
Date : 2025/07/20

This file defines the Python class of the ros2 node used to detect TurtleBot3 robots from a video frame.
We are leveraging the server's Nvidia GPU to run a custom inference model trained for YOLO.
The training dataset consisted of a selection of screenshots of the robot in the Gazebo simulation.

Copyright (c) Nicolas Erbetti

"""

##################################################################################################
# # DEPENDENCIES
# Python libraries
from typing import Any, List, Union, Optional
from ultralytics import YOLO
from ultralytics.utils import LOGGER
import cv2
import logging
import numpy as np
import torch
import threading
# Dedicated ros2 libraries
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
import rclpy
# Custom-made libraries.
##################################################################################################



##################################################################################################
# # VARIABLES
# Global VARIABLES
# Others
LOGGER.setLevel(logging.WARNING) # Silence Ultralytics logs
##################################################################################################


##################################################################################################
class RobotSegmentation(Node):
    """
    Node handling image segmentation using YOLO.

    Args:
        _node_name (str): Name of the ROS node.
    """
    ##############################################################################################
    # # # CLASS CONSTRUCTOR

    def __init__(self, _node_name: str):
        """
        Initialize the Segmentation node.

        Args:
            _node_name (str): Name of the ROS node.

        Returns:
            RobotSegmentation: A new, initialized class of this node.

        """

        # # INITIALIZING EVERY ATTRIBUTES
        # Initialize the parent classes.
        super().__init__(_node_name)
        # Initialize the dynamic attributes.
        self._cv_bridge = CvBridge()
        self._is_new_input_available = False
        self._input_img_cv = np.array(np.zeros((0, 0, 3), dtype=np.uint8))
        self._segmented_img_cv = np.array(np.zeros((0, 0, 3), dtype=np.uint8))
        # Initialize the ros parameters attributes.
        self.declare_parameters(
            namespace='',
            parameters=[
                ('run_rate', 10.0),
                ('auto_enable', True),
                ('model_path', ''),
                ('tb3_conf_threshold', 0.25),
            ]
        )
        # Initialize the attributes depending on ros parameters.
        self._yolo_model = YOLO(self.get_parameter("model_path").value, task="segment")
        # Initialize the messages.
        self._input_frame_msg = CompressedImage()
        self._segmented_frame_msg = Image()
        # Initialize the quality of service.
        self._qos = QoSProfile(
            depth=1, # Only keeps the latest frame in its buffer.
            reliability=QoSReliabilityPolicy.BEST_EFFORT, # Don’t bother resending dropped messages.
            history=QoSHistoryPolicy.KEEP_LAST # ensures that you're only keeping the most recent depth number of messages.
        )
        # Initialize the publishers.
        self._segmented_frame_pub = self.create_publisher(Image, "~/segmented_frame", self._qos)
        # Initialize the subscribers.
        self._input_frame_sub = self.create_subscription(CompressedImage, "input_frame", self.process_input_frame_cb, self._qos)
        # Initialize the ros timers.
        # Initialize the services.
        # Initialize the actions.
        # Initialize ros dynamic parameters.
        self.add_on_set_parameters_callback(self.process_parameters_cb)
        # Log relevant informations.
        if (torch.cuda.is_available() and torch.cuda.get_device_name(0)) : self.get_logger().info("GPU detected. The YOLO model will run on " + torch.cuda.get_device_name(0))
        else : self.get_logger().info("No GPU detected. Is CUDA installed ? The YOLO model will run on CPU.")

	##################################################################################################



	##################################################################################################
    # # GENERIC FUNCTIONS

    def run(self) -> int:
        """
        Main processing loop: decode, infer, encode, and publish.

        Args:

        Returns:
            int: 0 when the main loop is cut.
        """
        # Create the run rate.
        rate = self.create_rate(self.get_parameter("run_rate").value)
        # Run while the node is alive.
        while rclpy.ok():
            # 1) Wait for new input
            if (self._is_new_input_available):
                # 2) Decode the input image.
                self._input_img_cv = self.decode_compressed_image(self._input_frame_msg)             
                # 3) Run the inference.
                self._segmented_img_cv = self.run_inference(self._yolo_model, self._input_img_cv, self.get_parameter("tb3_conf_threshold").value)
                # 4) Encode this picture to a ros CompressedImage message.
                self._segmented_frame_msg = self.encode_uncompressed_image(self._segmented_img_cv)
                # 5) Publish them on the network.
                self.publish_pictures()
                # 6) Reset the flag for next frame
                self._is_new_input_available = False
            # 7) Wait for the rate duration.
            rate.sleep()
        # Return when ros is shutdown.
        return 0
    
	#################################################################################################



    ##################################################################################################
    # # SPECIFIC FUNCTIONS

    def encode_compressed_image(self, img_cv: np.array) -> CompressedImage:
        """!
        Encode the input uncompressed cv image to a ros image.

        Args:
            img_cv (np.ndarray): The input image.

        Returns:
            img_msg (CompressedImage): The encoded output CompressedImage structure.
        """
        # Initialize return variable.
        img_msg = CompressedImage()
        # Check if the input images are empty.
        if img_cv.shape[0] == 0 or img_cv.shape[1] == 0: return img_msg
        # Convert segmented images to ROS messages.
        success, encoded_image = cv2.imencode('.jpg', img_cv)
        # Return in case of failure.
        if not success: return img_msg
        # Fill the message.
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.format = "jpeg"
        img_msg.data = bytes(memoryview(encoded_image))
        # Return the encoded CompressedImage.
        return img_msg

    def encode_uncompressed_image(self, img_cv: np.ndarray) -> Image:
        """
        Convert OpenCV image to ROS Image message.

        Args:
            img_cv (np.ndarray): The image to convert.

        Returns:
            Image: ROS Image message.
        """
        # Safeguard.
        if img_cv.shape[0] == 0 or img_cv.shape[1] == 0:
            # Return an empty message if input image is invalid
            return Image()  
        # Fill the message.
        img_msg = self._cv_bridge.cv2_to_imgmsg(img_cv, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "segmentation_frame"
        # Return the message.
        return img_msg

    def decode_compressed_image(self, img_msg: CompressedImage) -> np.array:
        """!
        Decode the input compressed ros image to an opencv image.

        Args:
            img_msg (CompressedImage): The encoded input CompressedImage structure.

        Returns:
            img_cv (np.ndarray): The output image.
        """
        # Initialize return value.
        img_cv = np.array(np.zeros((0, 0, 3), dtype=np.uint8))
        # Safeguards: check if the input image is empty.
        if (len(img_msg.data) == 0):
            if self._is_debug_enabled: self.get_logger().warn("Input image is empty.")
            return img_cv
        # Register the compressed image data in a numpy array for computationnal efficiency.
        np_arr = np.frombuffer(img_msg.data, np.uint8)
        # Decode the image with opencv.
        img_cv = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
        # Return the decoded image.
        return img_cv

    def run_inference(self, model, input_img_cv: np.ndarray, conf: float) -> Union[List[Any], Any]:
        """
        Perform segmentation inference on a preprocessed image.

        Args:
            model (Yolo Model): The yolo model trained to recognize TurtleBots3.
            input_img_cv (np.ndarray): Input BGR image array.
            conf (float): The confidence above which we consider the segmentation as true.

        Returns:
            segmented_img_cv (np.ndarray): Output BGR image array with the segmentation results.
        """
        # Safeguards: check if the input image is empty.
        if (len(input_img_cv.data) == 0):
            # Log the warning.
            self.get_logger().warn("Input image is empty.")
            # Return the input image.
            return input_img_cv
        # Try/Catch the inference.
        try:
            # Run the inference.
            results = model.predict(
                source=input_img_cv,      # Pass RGB image directly.
                conf=conf,                # Set the confidence threshold.
                save=False,               # Don't save to disk.
                save_txt=False,           # Don't save annotations.
                save_crop=False           # Don't save cropped objects.
            )
            # Retrieve the (single, because we only passed one image to the model) result of the prediction.
            segmented_img_cv = results[0].plot()
            # Return the image associated with the result.
            return segmented_img_cv
        except Exception as e:
            # Log the error.
            self.get_logger().error(f"Inference failed: {e}")
            # Return the same image as input if an exception was raised during the inference.
            return input_img_cv
    
    def publish_pictures(self):
        """
        Publish the annotated pictures on the ros network.

        Args:

        Returns:
        """
        # Publish the segmented frame.
        self._segmented_frame_pub.publish(self._segmented_frame_msg)
        
    ##################################################################################################



	##################################################################################################
	# # ROS CALLBACK FUNCTIONS

	# SUBSCRIBERS CALLBACK
    def process_input_frame_cb(self, msg: CompressedImage) -> None:
        """
        ROS subscriber callback for incoming compressed image messages.

        Args:
            msg (CompressedImage): The latest received compressed image.
        """        
        # Register the image.
        self._input_frame_msg = msg
        # Set the new flag.
        self._is_new_input_available = True

    # TIMERS CALLBACK

    # SERVICES CALLBACK

    # ACTIONS CALLBACK

    # PARAMETERS CALLBACK
    def process_parameters_cb(self, params: List) -> SetParametersResult:
        """
        ROS callback invoked when dynamic parameters are set or changed.

        Iterates through each updated parameter, logs its new value, and
        updates the corresponding internal attribute.

        Args:
            params (List): List of parameters that were changed.

        Returns:
            SetParametersResult: Indicates whether all updates were successful.
        """
        # Iterate through each param.
        for param in params:
            # Log the new parameter.
            self.get_logger().info(f"Parameter change: {param.name} = {param.value}")
        # Return.
        return SetParametersResult(successful=True)
    
    ##################################################################################################



##################################################################################################
# # EXECUTABLE

# Main function
def main(args: Optional[List[str]] = None) -> int:
    """
    Entry point for the yolo segmentation node executable.

    Initializes the ROS client library, creates and spins the Segmentation node
    in a background thread, then runs its main loop. Cleans up resources on exit.

    Args:
        args (Optional[List[str]]): Command-line arguments to pass to rclpy.init.

    Returns:
        int: Exit status code (0 indicates normal shutdown).
    """
    # Initialize ros.
    rclpy.init(args=args)
    # Initialize the class.
    node = RobotSegmentation("yolo_segmentation")
    node.get_logger().info(f"Node initialized.")
    # Spin it in another thread.
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()
    # Handle the node's lifecycle.
    try:
        # Start the main loop.
        node.run()
    except KeyboardInterrupt:
        # Catch the termination signal.
        node.get_logger().info("Shutting down node.")
    finally:
        # Clean up.
        node.destroy_node()
        rclpy.shutdown()
        thread.join()
    # Exit.
    return 0

# Main guard
if __name__ == '__main__':
    main()

##################################################################################################