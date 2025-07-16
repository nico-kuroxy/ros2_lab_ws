/**********************************************************************************************************************/
//   author: nicolas.erbetti.k@gmail.com
//   brief: This file defines the header of the base node used to process the video stream.
//   copyright: © 2025 Nicolas Erbetti. All rights reserved.
/**********************************************************************************************************************/

#ifndef _PERCEPTION_IMAGE_SEGMENTATION_HPP_
#define _PERCEPTION_IMAGE_SEGMENTATION_HPP_

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> DEPENDENCIES
//> C/C++ libraries
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
//> ROS dedicated libraries
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/compressed_image.h>
#include <sensor_msgs/msg/image.h>
//> Custom-made libraries
#include <library/ros2_node.hpp>
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> GLOBAL VARIABLES
//> ATOMIC VARIABLES
//> OTHERS
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> CLASS DECLARATION
class ImageSegmentation : public Ros2Node {
/* A class to process images coming from the ros network. */

 public:
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> CONSTRUCTORS
    /* Constructor of the class. */
    ImageSegmentation(std::string _node_name);  // NOLINT
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> DESTRUCTORS
    /* Destructor of the class. */
    ~ImageSegmentation();
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> GENERIC FUNCTIONS / Used in every class.
    /* Function that resets all the ros messages.
       Declared as simply virtual to have a simple implementation that can be overriden. */
    virtual void resetRosMessages();
    /* Function that populate the output ros messages with a compression. */
    int compressFrame(const std::shared_ptr<rclcpp::Node>& _node_ptr, std::string _name, cv::UMat& _output_frame_cv, sensor_msgs::msg::CompressedImage& _output_frame_msg, bool _verbose);
    /* Function that convert an input opencv message to an output ros messages. */
    int convertFrame(const std::shared_ptr<rclcpp::Node>& _node_ptr, std::string _name, cv::UMat& _output_frame_cv, sensor_msgs::msg::Image& _output_frame_msg, bool _verbose);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> ACCESSORS
    //> GETTERS
    //> SETTERS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> SPECIFIC FUNCTIONS / Used only in this node.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 protected:
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> GENERAL ATTRIBUTES
    //> DYNAMIC / They can change during runtime.
    /* Whether or not a new image is available to be processed. */
    bool new_msg_;
    /* The converter between the ros images and the opencv images. */
    cv_bridge::CvImagePtr ros2cv_image_ptr_;
    /* The current input frame. */
    cv::UMat input_frame_cv_;
    /* The current output frame. */
    cv::UMat output_frame_cv_;
    //> FIXED / They cannot change during runtime. Declared as const.
    //> ROS PARAM / They are retrieved from the ros parameters server.
    /* The name of the camera from which the pictures are retrieved. */
    std::string camera_name_;
    /* The compression quality of the picture we are sending back to the network in debug mode. */
    int compression_quality_;
    //> PARAM-DEPENDENT / They are initialized based on the ros parameters.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> ROS ATTRIBUTES
    //> NODE HANDLER
    //> CHRONOMETERS
    //> MESSAGES
    /* The latest input frame coming from the camera, registered as a ros message. */
    sensor_msgs::msg::Image input_frame_msg_;
    /* The latest output frame of the pipeline, registered as a ros message. */
    sensor_msgs::msg::CompressedImage output_frame_msg_;
    //> PUBLISHERS
    /* Publish the segmented image if we are in debug mode. */
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr output_frame_pub_;
    //> SUBSCRIBERS
    /* Subscribe to the latest undistorted frame available on the ros network. */
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr input_frame_sub_;
    //> TIMERS
    //> SERVICES
    //> ACTIONS
    //> DYNAMIC RECONFIGURE SERVER
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> ROS CALLBACK FUNCTIONS
    //> SUBSCRIBERS CALLBACKS
    /* A function callback to retrieve the latest image published on the network.  */
    void processInputFrameCb(const sensor_msgs::msg::Image& _msg);
    //> TIMERS CALLBACK
    //> SERVICES
    //> ACTIONS
    //> DYNAMIC RECONFIGURE SERVER
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif  // _PERCEPTION_IMAGE_SEGMENTATION_HPP_
