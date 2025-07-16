/**********************************************************************************************************************/
//   author: nicolas.erbetti.k@gmail.com
//   brief: This file defines the header of the node used to detect an April Tag from a video frame.
//   copyright: © 2025 Nicolas Erbetti. All rights reserved.
/**********************************************************************************************************************/

#ifndef _PERCEPTION_APRIL_TAG_SEGMENTATION_HPP_
#define _PERCEPTION_APRIL_TAG_SEGMENTATION_HPP_

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> DEPENDENCIES
//> C/C++ libraries
#include <opencv2/opencv.hpp>
#include <string>
//> ROS dedicated libraries
#include <apriltag/apriltag.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/apriltag_pose.h>
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
//> Custom-made libraries
#include <perception/image_segmentation.hpp>
#include <interfaces/msg/status_apriltag_segmentation.hpp>
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> GLOBAL VARIABLES
//> ATOMIC VARIABLES
//> OTHERS
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> CLASS DECLARATION
class ApriltagSegmentation : public ImageSegmentation {
/* A class to detect april tags from images coming from the ros network. */

 public:
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> CONSTRUCTORS
    /* Constructor of the class. */
    ApriltagSegmentation(std::string _node_name);  // NOLINT
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> DESTRUCTORS
    /* Destructor of the class. */
    ~ApriltagSegmentation();
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> GENERIC FUNCTIONS / Used in every class.
    /* Function that handles the main loop of the node. */
    int execute();
    /* A function enabling or disabling the node depending on the value of the passed argument. */
    int enableNode(bool _is_node_enabled);
    /* Function that takes care of publishing the ros messages. */
    void publishAll();
    /* Function that resets all the ros messages. */
    void resetRosMessages();
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> ACCESSORS
    //> GETTERS
    //> SETTERS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> SPECIFIC FUNCTIONS / Used only in this node.
    /* Function to convert the april tag pose to retrieve the relative cartesian pose of the camera. */
    geometry_msgs::msg::Point convertCartesianPose(apriltag_pose_t& _pose);
    /* Function to convert the april tag pose to retrieve the relative angular pose of the camera. */
    geometry_msgs::msg::Point convertQuaternion2AngularPose(geometry_msgs::msg::Quaternion& _orientation);
    /* Function to convert the april tag pose to retrieve the relative angular pose of the camera. */
    geometry_msgs::msg::Quaternion convertAngularPose2Quaternion(geometry_msgs::msg::Point& _orientation);
    /* Function to convert the april tag pose to retrieve the quaternion orientation of the camera. */
    geometry_msgs::msg::Quaternion convertApriltagOrientation2Quaternions(apriltag_pose_t& _pose);
    /* Function to populate a PoseStamped message with the content of an april tag structure. */
    geometry_msgs::msg::PoseStamped createPoseMsg(const std::shared_ptr<rclcpp::Node>& _node_ptr, std::string _camera_name, geometry_msgs::msg::Point& _position, geometry_msgs::msg::Quaternion& _orientation);
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 protected:
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> GENERAL ATTRIBUTES
    //> DYNAMIC / They can change during runtime.
    /* Whether or not we retrieved the camera info yet. */
    bool are_camera_info_retrieved_;
    /* Whether or not an april tag is detected in the current picture. */
    bool is_apriltag_detected_;
    /* The structure holding the information about the tag family. */
    apriltag_family_t* tag_family_;
    /* The structure holding the tag detector itself. */
    apriltag_detector_t* tag_detector_;
    /* The structure holding the information about the detector. */
    apriltag_detection_info_t* tag_info_;
    /* The detected april tag in the latest frame. */
    zarray_t* tag_detections_;
    /* The detected relative pose of the camera from the april tag. */
    apriltag_pose_t tag_pose_;
    /* The relative cartesian position of the camera from the april tag. */
    geometry_msgs::msg::Point relative_position_;
    /* The relative angular position of the camera from the april tag. */
    geometry_msgs::msg::Point relative_orientation_;
    /* The relative quaternion orientation of the camera from the april tag, in the frame of the april tag. */
    geometry_msgs::msg::Quaternion relative_orientation_quaternion_april_tag_;
    /* The relative quaternion orientation of the camera from the april tag, in the frame of the robot. */
    geometry_msgs::msg::Quaternion relative_orientation_quaternion_robot_;
    //> FIXED / They cannot change during runtime. Declared as const.
    //> ROS PARAM / They are retrieved from the ros parameters server.
    //> PARAM-DEPENDENT / They are initialized based on the ros parameters.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> ROS ATTRIBUTES
    //> NODE HANDLER
    //> CHRONOMETERS
    //> MESSAGES
    /* The message holding the latest relative pose computed by the april tag detector. */
    geometry_msgs::msg::PoseStamped camera_pose_msg_;
    //> PUBLISHERS
    /* Publish the status of the node, including the number of bags runnings, their names, their topics. */
    rclcpp::Publisher<interfaces::msg::StatusApriltagSegmentation>::SharedPtr status_pub_;
    /* Publish the pose of the camera relative to the april tag if we detect any. */
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_pub_;
    //> SUBSCRIBERS
    //Subscribe to the latest camera info available on the ros network. */
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    //> TIMERS
    //> SERVICES
    //> ACTIONS
    //> DYNAMIC RECONFIGURE SERVER
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> ROS CALLBACK FUNCTIONS
    //> SUBSCRIBERS CALLBACKS
    /* A function callback to retrieve the latest camera info published on the network.  */
    void processCameraInfoCb(const sensor_msgs::msg::CameraInfo& _msg);
    //> TIMERS CALLBACK
    /* A function callback linked to timer_status_ used to publish data from the node. */
    void processStatusTimerCb(const interfaces::msg::StatusCoreNode& _core_status_node_msg);
    //> SERVICES
    //> ACTIONS
    //> DYNAMIC RECONFIGURE SERVER
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif  // _PERCEPTION_APRIL_TAG_SEGMENTATION_HPP_
