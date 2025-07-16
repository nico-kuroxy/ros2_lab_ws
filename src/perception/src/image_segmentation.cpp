/**********************************************************************************************************************/
//   author: nicolas.erbetti.k@gmail.com
//   brief: This file defines the source of the base node used to process the video stream.
//   copyright: © 2025 Nicolas Erbetti. All rights reserved.
/**********************************************************************************************************************/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//> HEADER
#include <perception/image_segmentation.hpp>
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//> CONSTRUCTORS
ImageSegmentation::ImageSegmentation(std::string _node_name) :
    // Member assignment list : initializing node handle and attributes which depends on it.
    Ros2Node(_node_name) {
    //> INITIALIZING EVERY ATTRIBUTE.
    // Initialize the dynamic attributes.
    this->new_msg_ = false;
    this->ros2cv_image_ptr_ = cv_bridge::CvImagePtr();
    this->input_frame_cv_ = cv::UMat();
    // Initialize the param server attributes.
    if (this->retrieveRosParameter<std::string>(this->ros_parameters_, "camera_name")) throw std::runtime_error("ROS parameter error.");
    if (this->retrieveRosParameter<double>(this->ros_parameters_, "camera_offset")) throw std::runtime_error("ROS parameter error.");
    // Initialize the attributes depending on ros parameters.
    // Initialize the messages.
    // Initialize the publishers.
    // Initialize the subscribers.
    // Initialize the ros timers.
    // Initialize the services.
    // Initialize the action.
    // Initialize the dynamic reconfigure server.
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//> DESTRUCTORS
ImageSegmentation::~ImageSegmentation() {
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> GENERIC FUNCTIONS / Used in every node.
void ImageSegmentation::resetRosMessages() {
    //> Reset every messages.
    // Reset the input frame message.
    this->input_frame_msg_ = sensor_msgs::msg::Image();
    // Reset the output frame message.
    this->output_frame_msg_ = sensor_msgs::msg::CompressedImage();
}
int ImageSegmentation::compressFrame(const std::shared_ptr<rclcpp::Node>& _node_ptr, std::string _name, cv::UMat& _frame_cv, sensor_msgs::msg::CompressedImage& _output_frame_msg, bool _verbose) {
    // Check if the image is not empty.
    if (_frame_cv.empty()) {
        if (_verbose) RCLCPP_WARN(_node_ptr->get_logger(), "The [%s] frame to compress empty, we cannot compress it.", _name.c_str());
        return -1;
    }
    // Convert UMat to Mat before encoding.
    cv::Mat image;
    _frame_cv.copyTo(image);
    std::vector<uchar> buf;
    std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, this->compression_quality_};
    cv::imencode(".jpg", image, buf, compression_params);
    // Create and populate CompressedImage message.
    _output_frame_msg.header.stamp = _node_ptr->get_clock()->now();
    _output_frame_msg.header.frame_id = _name;
    _output_frame_msg.format = "jpeg";
    _output_frame_msg.data = buf;
    // Return.
    return 0;
}
int ImageSegmentation::convertFrame(const std::shared_ptr<rclcpp::Node>& _node_ptr, std::string _name, cv::UMat& _frame_cv, sensor_msgs::msg::Image& _output_frame_msg, bool _verbose) {
    // Check if the image is not empty.
    if (_frame_cv.empty()) {
        if (_verbose) RCLCPP_WARN(_node_ptr->get_logger(), "The [%s] frame to compress empty, we cannot compress it.", _name.c_str());
        return -1;
    }
    // Convert UMat to Mat before encoding.
    cv::Mat image;
    _frame_cv.copyTo(image);
    // Convert.
    std_msgs::msg::Header header;
    header.stamp = _node_ptr->get_clock()->now();
    header.frame_id = _name;
    _output_frame_msg = *cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    // Return.
    return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> ACCESSERS
//> GETTERS
//> SETTERS
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> SPECIFIC FUNCTIONS / Used only in this node.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> ROS CALLBACK FUNCTIONS
//> SUBSCRIBERS CALLBACKS
void ImageSegmentation::processInputFrameCb(const sensor_msgs::msg::Image& _msg) {
    // Update the new message flag.
    this->new_msg_ = true;
    // Register the latest image message.
    this->input_frame_msg_ = _msg;
}
//> TIMERS CALLBACKS
//> SERVICE
//> ACTIONS
//> DYNAMIC RECONFIGURE SERVER
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//> EXECUTABLE
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
