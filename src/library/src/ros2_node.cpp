/**********************************************************************************************************************/
//   author: nicolas.erbetti.k@gmail.com
//   brief: This file defines the source of the ros2 node class used to derive every other ros node of the system.
//   copyright: © 2025 Nicolas Erbetti. All rights reserved.
/**********************************************************************************************************************/



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> HEADER
#include <library/ros2_node.hpp>
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> CONSTRUCTORS
Ros2Node::Ros2Node(std::string _node_name) :
    // Member assignment list : initializing rclcpp Node and attributes which depends on it.
    Node(_node_name) {
    //> INITIALIZING EVERY ATTRIBUTE.
    // Initialize the dynamic attributes.
    this->ros_parameters_ = std::unordered_map<std::string, std::shared_ptr<Ros2ParameterInterface>>();
    this->is_node_enabled_ = false;
    this->is_drs_triggered_ = false;
    // Initialize the ros param attributes.
    if (this->retrieveRosParameter<double>(this->ros_parameters_, "run_rate", 0.0, 1000.0)) throw std::runtime_error("ROS parameter error.");
    if (this->retrieveRosParameter<bool>(this->ros_parameters_, "auto_enable")) throw std::runtime_error("ROS parameter error.");
    if (this->retrieveRosParameter<bool>(this->ros_parameters_, "debug")) throw std::runtime_error("ROS parameter error.");
    // Initialize the attributes depending on ros parameters.
    // Initilize the chronometers.
    this->date_instanciated_ = this->get_clock()->now();  // Corresponds to the current time.
    this->date_enabled_ = rclcpp::Time(0, 0, RCL_ROS_TIME);  // Correspond to a time of -1 second.
    // Initialize the messages.
    this->core_status_node_msg_ = interfaces::msg::StatusCoreNode();
    // Initialize the status timer.
    this->status_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1/this->getValue<double>(this->ros_parameters_["run_rate"])), std::bind(&Ros2Node::wrapProcessStatusTimerCb, this));
    // Initialize the node toggle service.
    this->enable_node_service_server_ =
        this->create_service<interfaces::srv::EnableNode>("enable_node", std::bind(&Ros2Node::enableNodeServiceCb, this, std::placeholders::_1, std::placeholders::_2));
    // Log.
    RCLCPP_INFO(this->get_logger(), "Node core initialized.");
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> DESTRUCTORS
Ros2Node::~Ros2Node() {
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> GENERIC FUNCTIONS / Used in every node.
int Ros2Node::run() {
    // Start the spinning thread.
    std::thread spin_thread([this]() {rclcpp::spin(shared_from_this());});
    // Initialize the run rate object.
    rclcpp::Rate run_rate(this->getValue<double>(this->ros_parameters_["run_rate"]));
    // Enable the node if the auto_enable parameter is set to true.
    // It is important to NOT call it in the constructor to let the derived class overload all virtual functions.
    this->wrapEnableNode(this->getValue<bool>(this->ros_parameters_["auto_enable"]));
    // Initialize the dynamic reconfigure server. Required to do it here to make sure that every other parameter has been declared.
    this->dynamic_param_handle_ = this->add_on_set_parameters_callback(
        std::bind(&Ros2Node::processParametersCallback, this, std::placeholders::_1));
    // Main loop :
    while (rclcpp::ok()) {
        // If the node is enabled...
        if (this->is_node_enabled_) {
            // We try to execute the code of the node once.
            if (this->execute()) break;
        }
        // Sleep for a while to control the loop rate.
        run_rate.sleep();
    }
    // Join the spin thread to ensure it finishes before exiting the function
    if (spin_thread.joinable()) {spin_thread.join();}
    return 0;
}
int Ros2Node::wrapEnableNode(bool _is_node_enabled) {
    // Wrap the call to the enableNode function.
    return this->enableNode(_is_node_enabled);
}
interfaces::msg::StatusCoreNode Ros2Node::createBaseStatusNodeMsg(
    rclcpp::Time _date_instanciated,  bool _is_node_enabled, rclcpp::Time _date_enabled, bool _is_debug_enabled) {
    // Initialize the return variable.
    interfaces::msg::StatusCoreNode msg;
    // Update the header.
    msg.header.stamp = this->get_clock()->now();
    // Fill the chronometer parameters.
    msg.date_instanciated = _date_instanciated;
    msg.is_node_enabled = _is_node_enabled;
    msg.date_enabled = _date_enabled;
    // Fill the other generic parameters.
    msg.is_debug_enabled = _is_debug_enabled;
    // Return.
    return msg;
}
template<typename T>
int Ros2Node::retrieveRosParameter(
    std::unordered_map<std::string, std::shared_ptr<Ros2ParameterInterface>>& _ros_parameters,
        const std::string _name_parameter, std::optional<T> _min_value, std::optional<T> _max_value) {  // NOLINT
    // Check if ros is running. This is required since this function is static and can be called outside of a ros node.
    if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Unable to access the ros network. Is roscore running ? Are the ip addresses set properly ?");
        return -1;
    }
    // Declare the parameter if it does not already exist.
    this->declare_parameter<T>(_name_parameter, T());
    // Register a parameter buffer.
    T buffer;
    rclcpp::Parameter ros_parameter;
    // Try to read the parameter.
    this->get_parameter(_name_parameter, ros_parameter);
    if (!this->get_node_parameters_interface()->get_parameter_overrides().count(_name_parameter)) {
        // Not set in YAML.
        RCLCPP_ERROR(this->get_logger(), "Failed to retrieve %s in YAML. Killing the node.", _name_parameter.c_str());
        rclcpp::shutdown();
        return -1;
    } else {
        // If present in the yaml, we update the buffer.
        this->get_parameter(_name_parameter, buffer);
    }
    // Create the parameter.
    std::shared_ptr<Ros2Parameter<T>> parameter = std::make_shared<Ros2Parameter<T>>(_name_parameter, _min_value, _max_value);
    // Try to set its value.
    if (parameter->setValue(buffer)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid value for %s. Killing the node.", _name_parameter.c_str());
        rclcpp::shutdown();
        return -1;
    }
    // Register the parameter.
    _ros_parameters.insert({_name_parameter, parameter});
    // Return.
    return 0;
}
template<typename T>
int Ros2Node::retrieveRosParameter(
    std::unordered_map<std::string, std::shared_ptr<Ros2ParameterInterface>>& _ros_parameters,
       const std::string _name_parameter, T* _attribute_ptr, std::optional<T> _min_value, std::optional<T> _max_value) {
    // Check if ros is running. This is required since this function is static and can be called outside of a ros node.
    if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Unable to access the ros network. Is roscore running ? Are the ip addresses set properly ?");
        return -1;
    }
    // Declare the parameter.
    this->declare_parameter(_name_parameter, T());
    // Try to read the parameter.
    rclcpp::Parameter ros_parameter;
    this->get_parameter(_name_parameter, ros_parameter);
    if (ros_parameter.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        // Not set in YAML.
        RCLCPP_ERROR(this->get_logger(), "Failed to retrieve %s in YAML. Killing the node.", _name_parameter.c_str());
        rclcpp::shutdown();
        return -1;
    } else {
        // If the parameter exists in the YAML, we load it.
        this->get_parameter(_name_parameter, *_attribute_ptr);
    }
    // Create the parameter.
    std::shared_ptr<Ros2Parameter<T>> parameter = std::make_shared<Ros2Parameter<T>>(_name_parameter, _min_value, _max_value);
    // Try to set its value.
    if (parameter->setValuePtr(_attribute_ptr)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid value for %s. Killing the node.", _name_parameter.c_str());
        rclcpp::shutdown();
        return -1;
    }
    // Register the parameter.
    _ros_parameters.insert({_name_parameter, parameter});
    // Return.
    return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> ACCESSERS
//> GETTERS
std::unordered_map<std::string, std::shared_ptr<Ros2ParameterInterface>> Ros2Node::getRosParameters() const {
    // Get the map of ros parameters.
    return this->ros_parameters_;
}
template<typename T>
T Ros2Node::getValue(const std::shared_ptr<Ros2ParameterInterface>& _parameter) {
    // Dynamically cast the interface to its proper type.
    auto typed_parameter = dynamic_cast<Ros2Parameter<T>*>(_parameter.get());
    // Check if the cast succeeded.
    if (!typed_parameter) throw std::bad_cast();
    // Otherwise, return the value.
    return typed_parameter->getValue();
}
//> SETTERS
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> SPECIFIC FUNCTIONS / Used only in this node.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> ROS CALLBACK FUNCTIONS
//> SUBSCRIBERS CALLBACKS
//> TIMERS CALLBACKS
void Ros2Node::wrapProcessStatusTimerCb() {
    // Create the core node status message.
    this->core_status_node_msg_ = this->createBaseStatusNodeMsg(
        this->date_instanciated_, this->is_node_enabled_, this->date_enabled_, this->getValue<bool>(this->ros_parameters_["debug"]));
    // Pass it to the actual status timer callback function.
    this->processStatusTimerCb(this->core_status_node_msg_);
}
//> SERVICES CALLBACKS
void Ros2Node::enableNodeServiceCb(
    std::shared_ptr<interfaces::srv::EnableNode::Request> _req,
        std::shared_ptr<interfaces::srv::EnableNode::Response> _res) {
    // Initialize the variables.
    _res->success = false;
    // Handle the request fields.
    int result = this->enableNode(_req->enable);
    std::string request_str = (_req->enable) ? "enabled" : "disabled";
    std::string response_str = (result) ? "failed" : "accepted";
    // Update the response fields.
    _res->success = (result) ? false : true;
    _res->response =
        std::string("Request ") + response_str + std::string(". The node is [") + request_str + std::string("].");
    // Log the result.
    RCLCPP_ERROR(this->get_logger(), "%s", _res->response.c_str());
    // Return.
    return;
}
//> ACTIONS CALLBACKS
//> DYNAMIC RECONFIGURE SERVER CALLBACK
rcl_interfaces::msg::SetParametersResult Ros2Node::processParametersCallback(const std::vector<rclcpp::Parameter>& _parameters) {
    // Initialize variables.
    rcl_interfaces::msg::SetParametersResult result;
    // Update the drs trigger flag.
    this->is_drs_triggered_ = true;
    // Prepare the result message.
    result.successful = true;
    result.reason = "success";
    // Iterate through each parameters of the node.
    for (const rclcpp::Parameter &param: _parameters) {
        // Log its newest value.
        RCLCPP_INFO(this->get_logger(), "Updating [%s] of type [%s] with value [%s]", param.get_name().c_str(), param.get_type_name().c_str(), param.value_to_string().c_str());
        // Check if the new value is valid within the scope of the node.
        // We need to use the overloaded setValue() function to allow the RosParameter class to automatically handle the type of the parameter.
        if (this->ros_parameters_[param.get_name()]->setValue(param)) {
            result.successful = false;
            result.reason = "Failed to update " + param.get_name() + ".";
        }
    }
    // Reset the drs trigger flag.
    this->is_drs_triggered_ = false;
    // Return result.
    return result;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> EXPLICIT TEMPLATE DECLARATION // Required for the compiler to see them without declaring them in the header file.
//> 1st overload of the retrieve function.
template int Ros2Node::retrieveRosParameter<bool>(std::unordered_map<std::string, std::shared_ptr<Ros2ParameterInterface>>& _ros_parameters, const std::string _name_parameter, std::optional<bool> _min_value, std::optional<bool> _max_value);  // NOLINT
template int Ros2Node::retrieveRosParameter<int>(std::unordered_map<std::string, std::shared_ptr<Ros2ParameterInterface>>& _ros_parameters, const std::string _name_parameter, std::optional<int> _min_value, std::optional<int> _max_value);  // NOLINT
template int Ros2Node::retrieveRosParameter<float>(std::unordered_map<std::string, std::shared_ptr<Ros2ParameterInterface>>& _ros_parameters, const std::string _name_parameter, std::optional<float> _min_value, std::optional<float> _max_value);  // NOLINT
template int Ros2Node::retrieveRosParameter<double>(std::unordered_map<std::string, std::shared_ptr<Ros2ParameterInterface>>& _ros_parameters, const std::string _name_parameter, std::optional<double> _min_value, std::optional<double> _max_value);  // NOLINTNT
template int Ros2Node::retrieveRosParameter<std::string>(std::unordered_map<std::string, std::shared_ptr<Ros2ParameterInterface>>& _ros_parameters, const std::string _name_parameter, std::optional<std::string> _min_value, std::optional<std::string> _max_value);  // NOLINTNT
//> 2nd overload of the retrieve function.
template int Ros2Node::retrieveRosParameter<bool>(std::unordered_map<std::string, std::shared_ptr<Ros2ParameterInterface>>& _ros_parameters, const std::string _name_parameter, bool* _attribute_ptr, std::optional<bool> _min_value, std::optional<bool> _max_value);  // NOLINT
template int Ros2Node::retrieveRosParameter<int>(std::unordered_map<std::string, std::shared_ptr<Ros2ParameterInterface>>& _ros_parameters, const std::string _name_parameter, int* _attribute_ptr, std::optional<int> _min_value, std::optional<int> _max_value);  // NOLINT
template int Ros2Node::retrieveRosParameter<float>(std::unordered_map<std::string, std::shared_ptr<Ros2ParameterInterface>>& _ros_parameters, const std::string _name_parameter, float* _attribute_ptr, std::optional<float> _min_value, std::optional<float> _max_value);  // NOLINT
template int Ros2Node::retrieveRosParameter<double>(std::unordered_map<std::string, std::shared_ptr<Ros2ParameterInterface>>& _ros_parameters, const std::string _name_parameter, double* _attribute_ptr, std::optional<double> _min_value, std::optional<double> _max_value);  // NOLINTNT
template int Ros2Node::retrieveRosParameter<std::string>(std::unordered_map<std::string, std::shared_ptr<Ros2ParameterInterface>>& _ros_parameters, const std::string _name_parameter, std::string* _attribute_ptr, std::optional<std::string> _min_value, std::optional<std::string> _max_value);  // NOLINTNT
//> 1st overload of the retrieve function.
template bool Ros2Node::getValue<bool>(const std::shared_ptr<Ros2ParameterInterface>& _parameter);  // NOLINT
template int Ros2Node::getValue<int>(const std::shared_ptr<Ros2ParameterInterface>& _parameter);  // NOLINT
template float Ros2Node::getValue<float>(const std::shared_ptr<Ros2ParameterInterface>& _parameter);  // NOLINT
template double Ros2Node::getValue<double>(const std::shared_ptr<Ros2ParameterInterface>& _parameter);  // NOLINT
template std::string Ros2Node::getValue<std::string>(const std::shared_ptr<Ros2ParameterInterface>& _parameter);  // NOLINT
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
