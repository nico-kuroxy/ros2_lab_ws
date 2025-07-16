/**********************************************************************************************************************/
//   author: nicolas.erbetti.k@gmail.com
//   brief: This file defines the header of the node used to detect an April Tag from a video frame.
//   copyright: © 2025 Nicolas Erbetti. All rights reserved.
/**********************************************************************************************************************/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//> HEADER
#include <perception/apriltag_segmentation.hpp>
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//> CONSTRUCTORS
ApriltagSegmentation::ApriltagSegmentation(std::string _node_name) :
    // Member assignment list : initializing node handle and attributes which depends on it.
    ImageSegmentation(_node_name) {
    //> INITIALIZING EVERY ATTRIBUTE.
    // Initialize the dynamic attributes.
    this->tag_info_ = new apriltag_detection_info_t();
    this->tag_family_ = tagStandard41h12_create();
    this->tag_detector_ = apriltag_detector_create();
    apriltag_detector_add_family(tag_detector_, tag_family_);
    this->relative_position_ = geometry_msgs::msg::Point();
    this->relative_orientation_ = geometry_msgs::msg::Point();
    this->relative_orientation_quaternion_april_tag_ = geometry_msgs::msg::Quaternion();
    // Initialize the param server attributes.
    if (this->retrieveRosParameter<int>(this->ros_parameters_, "camera_position")) throw std::runtime_error("ROS parameter error.");
    if (this->retrieveRosParameter<double>(this->ros_parameters_, "tag_size", &this->tag_info_->tagsize, 0.0, 10.0)) throw std::runtime_error("ROS parameter error.");
    if (this->retrieveRosParameter<int>(this->ros_parameters_, "nthreads", &this->tag_detector_->nthreads, 0, 10)) throw std::runtime_error("ROS parameter error.");
    if (this->retrieveRosParameter<bool>(this->ros_parameters_, "refine_edges", &this->tag_detector_->refine_edges)) throw std::runtime_error("ROS parameter error.");
    if (this->retrieveRosParameter<float>(this->ros_parameters_, "quad_sigma", &this->tag_detector_->quad_sigma, 0.0, 10.0)) throw std::runtime_error("ROS parameter error.");
    if (this->retrieveRosParameter<float>(this->ros_parameters_, "quad_decimate", &this->tag_detector_->quad_decimate, 0.0, 10.0)) throw std::runtime_error("ROS parameter error.");
    // Initialize the attributes depending on ros parameters.
    // Initialize the messages.
    // Handled in the enableNode() function.
    // Initialize the publishers.
    this->status_pub_ =
      this->create_publisher<interfaces::msg::StatusApriltagSegmentation>("~/status", 10);
    // Initialize the subscribers.
    // Initialize the ros timers.
    // Initialize the services.
    // Initialize the action.
    // Initialize the dynamic reconfigure server.
    // Wrap the initiliazation of the node.
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//> DESTRUCTORS
ApriltagSegmentation::~ApriltagSegmentation() {
  // Destroying the april tag elements.
  apriltag_detector_destroy(this->tag_detector_);
  tagStandard41h12_destroy(this->tag_family_);
  zarray_destroy(this->tag_detections_);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> GENERIC FUNCTIONS / Used in every node.
int ApriltagSegmentation::execute() {
  // Initialize variables.
  // Execute the april tag detection pipeline.
  // Safeguard : check if we have new message to process.
  if (this->new_msg_ && !this->input_frame_msg_.data.empty() && this->are_camera_info_retrieved_) {
    // Decode the inpout image as a gray picture.
    this->ros2cv_image_ptr_ = cv_bridge::toCvCopy(this->input_frame_msg_, sensor_msgs::image_encodings::MONO8);
    // Copy the decoded picture as an opencv matrix.
    this->ros2cv_image_ptr_->image.copyTo(this->input_frame_cv_);
    // Convert the opencv matrix to the apriltag format. The tag frame cannot be an attribute of the class since it requires const attribute to be initialized upon construction.
    image_u8_t tag_frame = {.width = this->input_frame_cv_.cols, .height = this->input_frame_cv_.rows, .stride = this->input_frame_cv_.cols, .buf = this->input_frame_cv_.getMat(cv::ACCESS_READ).data};
    this->tag_detections_ = apriltag_detector_detect(this->tag_detector_, &tag_frame);
    // Update the detection flag.
    this->is_apriltag_detected_ = zarray_size(this->tag_detections_);
    // Issue a warning if we see multiple of them.
    if (zarray_size(this->tag_detections_) > 1) RCLCPP_WARN(this->get_logger(), "Multiple apriltag detected [%d]. Unexpected behavior might occur.", zarray_size(this->tag_detections_));
    // Prepare the output image for visualization purpose.
    cv::cvtColor(this->input_frame_cv_, this->output_frame_cv_, cv::COLOR_GRAY2BGR);
    // Process apriltags.
    for (int i = 0; i < zarray_size(this->tag_detections_); i++) {
        // Assign the detected tag.
        zarray_get(this->tag_detections_, i, &this->tag_info_->det);
        // Estimate pose.
        estimate_tag_pose(this->tag_info_, &this->tag_pose_);
        // Retrieve the relative pose.
        this->relative_position_ = this->convertCartesianPose(this->tag_pose_);
        // Retrieve the quaternion angle.
        this->relative_orientation_quaternion_april_tag_ = this->convertApriltagOrientation2Quaternions(this->tag_pose_);
        // Retrieve the relative euler angles in the proper frame.
        this->relative_orientation_ = this->convertQuaternion2AngularPose(this->relative_orientation_quaternion_april_tag_);
        // Express the relative angle as quaternion in the robot frame.
        this->relative_orientation_quaternion_robot_ = this->convertAngularPose2Quaternion(this->relative_orientation_);
        // Create pose message. This needs to be done here otherwise the node crashes.
        this->camera_pose_msg_ = createPoseMsg(shared_from_this(), this->getValue<std::string>(this->ros_parameters_["camera_name"]), this->relative_position_, this->relative_orientation_quaternion_robot_);
        // Outline the april tag.
        if (this->getValue<bool>(this->ros_parameters_["debug"])) {
            // Draw the detection outline
            for (int j=0; j < 4; j++) {
                cv::line(this->output_frame_cv_, 
                    cv::Point(this->tag_info_->det->p[j][0], this->tag_info_->det->p[j][1]), 
                    cv::Point(this->tag_info_->det->p[(j+1)%4][0], this->tag_info_->det->p[(j+1)%4][1]), 
                    cv::Scalar(this->tag_info_->det->id*75, this->tag_info_->det->id*75, 100), 2);
            }
            // Draw the ID of the tag
            cv::putText(this->output_frame_cv_, 
                        "Tag " + std::to_string(this->tag_info_->det->id), 
                        cv::Point(this->tag_info_->det->c[0] - 30, this->tag_info_->det->c[1]), 
                        cv::FONT_HERSHEY_PLAIN, 0.5, cv::Scalar(this->tag_info_->det->id*75, this->tag_info_->det->id*75, 100), 1);
            // Draw the x pose of the tag.
            cv::putText(this->output_frame_cv_, "x " + std::to_string(this->relative_position_.x), 
                cv::Point(this->tag_info_->det->c[0] - 30, this->tag_info_->det->c[1] + 10), 
                cv::FONT_HERSHEY_PLAIN, 0.5, cv::Scalar(this->tag_info_->det->id*75, this->tag_info_->det->id*75, 100), 1);
            // Draw the y pose of the tag.
            cv::putText(this->output_frame_cv_, "y " + std::to_string(this->relative_position_.y), 
                cv::Point(this->tag_info_->det->c[0] - 30, this->tag_info_->det->c[1] + 20), 
                cv::FONT_HERSHEY_PLAIN, 0.5, cv::Scalar(this->tag_info_->det->id*75, this->tag_info_->det->id*75, 100), 1);
            // Draw the theta pose of the tag.
            cv::putText(this->output_frame_cv_, "theta " + std::to_string(this->relative_orientation_.z), 
                cv::Point(this->tag_info_->det->c[0] - 30, this->tag_info_->det->c[1] + 30), 
                cv::FONT_HERSHEY_PLAIN, 0.5, cv::Scalar(this->tag_info_->det->id*75, this->tag_info_->det->id*75, 100), 1);
        }
    }
    // Prepare the lines picture for the ros message.
    this->compressFrame(shared_from_this(), this->getValue<std::string>(this->ros_parameters_["camera_name"]), this->output_frame_cv_, this->output_frame_msg_, (this->getValue<bool>(this->ros_parameters_["debug"])));
    // Publish all the information on the network.
    this->publishAll();
  }
  return 0;
}
int ApriltagSegmentation::enableNode(bool _is_node_enabled) {
    // Initialize return variable.
    int node_toggled = -1;
    // If we want to enable the node.
    if (_is_node_enabled) {
        if (this->is_node_enabled_) {
            RCLCPP_ERROR(this->get_logger(), "The node is already enabled !");
        } else {
            // Log info.
            RCLCPP_INFO(this->get_logger(), "Enabling the node...");
            // Prepare the return value.
            node_toggled = 0;
            // Reset the ros messages. We need to do it before the advertisement to prepare any new message coming up.
            this->resetRosMessages();
            // Advertise every publisher.
            this->camera_pose_pub_ =
                this->create_publisher<geometry_msgs::msg::PoseStamped>("~/apriltag_pose", 1);
            this->output_frame_pub_ =
                this->create_publisher<sensor_msgs::msg::CompressedImage>("~/apriltag_frame", 1);
            // Subscribe every subscriber.
            this->input_frame_sub_ =
                this->create_subscription<sensor_msgs::msg::Image>("input_frame", 1,
                std::bind(&ApriltagSegmentation::processInputFrameCb, this, std::placeholders::_1));
            this->camera_info_sub_ =
                this->create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", 1,
                std::bind(&ApriltagSegmentation::processCameraInfoCb, this, std::placeholders::_1));
            // Start every timer.
            // Deploy every service.
            // Toggle the flag.
            this->is_node_enabled_ = true;
            // Wrap up.
            RCLCPP_INFO(this->get_logger(), "Node enabled.");
        }
    } else {
    // If we want to disable the node.
        if (!this->is_node_enabled_) {
            RCLCPP_ERROR(this->get_logger(), "The node is already disabled !");
        } else {
            // Log info.
            RCLCPP_INFO(this->get_logger(), "Disabling the node...");
            // Prepare the return value.
            node_toggled = 0;
            // Toggle the flag.
            this->is_node_enabled_ = false;
            // Unadvertise every publisher, even those used for debugging.
            this->output_frame_pub_.reset();
            // Unsubscribe every subscriber.
            this->input_frame_sub_.reset();
            this->camera_info_sub_.reset();
            // Stop every timer.
            // Remove every service.
            // Reset the ros messages. We need to do it after the shutdown to prevent any new message to come up.
            this->resetRosMessages();
            // Wrap up.
            RCLCPP_INFO(this->get_logger(), "Node disabled.");
        }
    }
    // Return.
    return node_toggled;
}
void ApriltagSegmentation::resetRosMessages() {
    //> Reset every messages.
    // Reset the input frame message.
    this->input_frame_msg_ = sensor_msgs::msg::Image();
    // Reset the output frame message.
    this->output_frame_msg_ = sensor_msgs::msg::CompressedImage();
    // Reset the flags related to the input messages.
    this->new_msg_ = false;
    this->are_camera_info_retrieved_ = false;
    this->is_apriltag_detected_ = false;
}
void ApriltagSegmentation::publishAll() {
    // Publish every non-debug-related messages.
    if (this->is_apriltag_detected_) this->camera_pose_pub_->publish(this->camera_pose_msg_);
    // Publish every debug-related messages.
    if (this->getValue<bool>(this->ros_parameters_["debug"])) this->output_frame_pub_->publish(this->output_frame_msg_);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> ACCESSERS
//> GETTERS
//> SETTERS
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> SPECIFIC FUNCTIONS / Used only in this node.
geometry_msgs::msg::Point ApriltagSegmentation::convertCartesianPose(apriltag_pose_t& _pose) {
    // Create the return message.
    geometry_msgs::msg::Point relative_position;
    // Fill the position.
    relative_position.x = this->getValue<int>(this->ros_parameters_["camera_position"]) * _pose.t->data[2];
    relative_position.y = - this->getValue<int>(this->ros_parameters_["camera_position"]) * _pose.t->data[0] + this->getValue<double>(this->ros_parameters_["camera_offset"]);
    relative_position.z = - this->getValue<int>(this->ros_parameters_["camera_position"]) * _pose.t->data[1];
    // Return.
    return relative_position;
}
geometry_msgs::msg::Point ApriltagSegmentation::convertQuaternion2AngularPose(geometry_msgs::msg::Quaternion& _orientation) {
    // Create the return message.
    geometry_msgs::msg::Point relative_angle;
    // Convert the quaternion to euler angles.
    tf2::Quaternion q(_orientation.x, _orientation.y, _orientation.z, _orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    // Fill the angles.
    relative_angle.x = -yaw;
    relative_angle.y = roll;
    relative_angle.z = pitch;
    // Return.
    return relative_angle;
}
geometry_msgs::msg::Quaternion ApriltagSegmentation::convertAngularPose2Quaternion(geometry_msgs::msg::Point& _orientation) {
    // Create the return message.
    geometry_msgs::msg::Quaternion relative_angle;
    // Convert the euler angles to quaternions.
    tf2::Quaternion q;
    q.setRPY(_orientation.x, _orientation.y, _orientation.z);
    // Fill the angles.
    relative_angle.x = q.x();
    relative_angle.y = q.y();
    relative_angle.z = q.z();
    relative_angle.w = q.w();
    // Return.
    return relative_angle;
}
geometry_msgs::msg::Quaternion ApriltagSegmentation::convertApriltagOrientation2Quaternions(apriltag_pose_t& _pose) {
    // Create the return message.
    geometry_msgs::msg::Quaternion q_msg;
    // Convert rotation matrix to quaternion
    tf2::Quaternion q;
    cv::Mat R(3, 3, CV_64F, _pose.R->data);
    cv::Vec3d rvec;
    cv::Rodrigues(R, rvec);
    // Convert to tf Quaternion
    tf2::Matrix3x3 tf_R(
    R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
    R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
    tf_R.getRotation(q);
    q.normalize();
    // Fill the message.
    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();
    // Return.
    return q_msg;
}
geometry_msgs::msg::PoseStamped ApriltagSegmentation::createPoseMsg(const std::shared_ptr<rclcpp::Node>& _node_ptr, std::string _camera_name, geometry_msgs::msg::Point& _position, geometry_msgs::msg::Quaternion& _orientation) {
    // Create the return message.
    geometry_msgs::msg::PoseStamped pose_msg;
    // Fill the header.
    pose_msg.header.stamp = _node_ptr->get_clock()->now();
    pose_msg.header.frame_id = _camera_name;
    // Fill the position.
    pose_msg.pose.position = _position;
    // Fill the orientation.
    pose_msg.pose.orientation = _orientation;
    // Return.
    return pose_msg;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> ROS CALLBACK FUNCTIONS
//> SUBSCRIBERS CALLBACKS
void ApriltagSegmentation::processCameraInfoCb(const sensor_msgs::msg::CameraInfo& _msg) {
    // Update the tag info structure if it has the proper size.
    if (_msg.k.size() == 9) {
        if (!this->are_camera_info_retrieved_) {
            this->are_camera_info_retrieved_ = true;
        }
        this->tag_info_->fx = _msg.k[0];
        this->tag_info_->fy = _msg.k[4];
        this->tag_info_->cx = _msg.k[2];
        this->tag_info_->cy = _msg.k[5];
    } else {
        RCLCPP_INFO(this->get_logger(), "Received intrinsic camera matrix has wrong size [%ld vs 9].", _msg.k.size());
    }
}
//> TIMERS CALLBACKS
void ApriltagSegmentation::processStatusTimerCb(const interfaces::msg::StatusCoreNode& _core_status_node_msg) {
  // Create the status message.
  interfaces::msg::StatusApriltagSegmentation msg;
  // Fill it with the base status node.
  msg.core_status = _core_status_node_msg;
  // Fill the rest of the status messages.
  msg.is_apriltag_detected = this->is_apriltag_detected_;
  // Publish.
  this->status_pub_->publish(msg);
}
//> SERVICE
//> ACTIONS
//> DYNAMIC RECONFIGURE SERVER
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//> EXECUTABLE
/* Main function of the executable running the node. */
int main(int argc, char **argv) {
  // Initialize the node.
  rclcpp::init(argc, argv);
  // Instanciate the class.
  auto node = std::make_shared<ApriltagSegmentation>("apriltag_segmentation");
  RCLCPP_INFO(node->get_logger(), "Node initialized.");
  // Start the main loop.
  node->run();
  // Exit.
  RCLCPP_INFO(node->get_logger(), "Node exited.");
  rclcpp::shutdown();
  return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
