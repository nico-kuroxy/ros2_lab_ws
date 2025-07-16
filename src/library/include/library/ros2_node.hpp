/**********************************************************************************************************************/
//   author: nicolas.erbetti.k@gmail.com
//   brief: This file defines the header of the ros2 node class used to derive every other ros node of the system.
//   copyright: © 2025 Nicolas Erbetti. All rights reserved.
/**********************************************************************************************************************/

#ifndef _LIBRARY_ROS2NODE_HPP_
#define _LIBRARY_ROS2NODE_HPP_

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> DEPENDENCIES
//> C/C++ libraries
#include <string>
#include <unordered_map>
//> ROS dedicated libraries
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
//> Custom-made libraries
#include <interfaces/msg/status_core_node.hpp>
#include <interfaces/srv/enable_node.hpp>
#include <library/ros2_parameter.hpp>
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> GLOBAL VARIABLES
//> ATOMIC VARIABLES
//> OTHER VARIABLES
//> NAMESPACES
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> CLASS DECLARATION
class Ros2Node : public rclcpp::Node {
/* A class to instanciate the base of a ros 2 node.
   It derives publicly from rclcpp::Node to retrieve its public and protected attributes. */

 public:
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> CONSTRUCTORS
    /* Constructor of the class. Declared as explicit to prevent implicit conversions. */
    explicit Ros2Node(std::string _node_name);  // NOLINT
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> DESTRUCTORS
    /* Destructor of the class. */
    ~Ros2Node();
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> GENERIC FUNCTIONS / Used in every node.
    /* Function that wraps the main loop of the node to keep track of the elapsed time at low level. */
    int run();
    /* Function that wrap the enableNode function to avoid compilation warnings due to the purely virtual function. */
    int wrapEnableNode(bool _is_node_enabled);
    /* Function that executes the sequence of operations required to have the node working.
       Virtual, so needs to be defined in derived classes. */
    virtual int execute() = 0;
    /* Function that enables or disables the node depending on the value of the passed argument.
       Virtual, so needs to be defined in derived classes depending on the existing ros attributes. */
    virtual int enableNode(bool _is_node_enabled) = 0;
    /* Function that creates a basic status node message based on the content of the class. */
    interfaces::msg::StatusCoreNode createBaseStatusNodeMsg(
      rclcpp::Time _date_instanciated, bool _is_node_enabled,  rclcpp::Time _date_enabled, bool _is_debug_enabled);
    /* Function that retrieves a ros parameter from the parameter server and set it directly as attribute.
       Declared as static to enable its usage from classes not derived from this one. */
   template<typename T>
   int retrieveRosParameter(std::unordered_map<std::string, std::shared_ptr<Ros2ParameterInterface>>& _ros_parameters,
         const std::string _name_parameter, std::optional<T> _min_value = std::nullopt, std::optional<T> _max_value = std::nullopt);
    /* Function that retrieves a ros parameter from the parameter server and set it directly as attribute but also links it to another attribute through its pointer.
       Declared as static to enable its usage from classes not derived from this one. */
   template<typename T>
   int retrieveRosParameter(std::unordered_map<std::string, std::shared_ptr<Ros2ParameterInterface>>& _ros_parameters,
         const std::string _name_parameter, T* _attribute_ptr, std::optional<T> _min_value = std::nullopt, std::optional<T> _max_value = std::nullopt);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> ACCESSERS
    //> GETTERS / All getters are declared as const since they are not modifying any class attributes.
    /* Get the map of ros parameters. */
    std::unordered_map<std::string, std::shared_ptr<Ros2ParameterInterface>> getRosParameters() const;
    /* Get the value from the Ros Parameter Interface. */
    template<typename T>
    T getValue(const std::shared_ptr<Ros2ParameterInterface>& _parameter);
    //> SETTERS / All setters return an int, 0 or -1 if the setting was successfull or not, respectively.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> SPECIFIC FUNCTIONS / Used only in this node.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 protected:
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> C/C++ ATTRIBUTES
    //> DYNAMIC ATTRIBUTES / They can change during runtime.
    /* Whether or not the node is enabled to run. */
    bool is_node_enabled_;
    /* Whether or not the dynamic reconfigure server was triggered and not yet processed. */
    bool is_drs_triggered_;
    //> FIXED ATTRIBUTES / They cannot change during runtime. Declared as static const.
    //> ROS PARAM ATTRIBUTES / They are retrieved from the ros parameters server.
    /* The dictionnary used to store the ros parameters. */
    std::unordered_map<std::string, std::shared_ptr<Ros2ParameterInterface>> ros_parameters_;
    //> PARAM-DEPENDANT ATTRIBUTES / They are initialized based on the ros parameters.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> ROS ATTRIBUTES
    //> CHRONOMETERS / Dates and duration used to keep track of time.
    /* The date at which the node was instanciated. */
    rclcpp::Time date_instanciated_;
    /* The date at which the node was enabled. */
    rclcpp::Time date_enabled_;
    //> MESSAGES / The messages that we subscribe or publish from/on the network.
    interfaces::msg::StatusCoreNode core_status_node_msg_;
    //> SUBSCRIBERS
    //> PUBLISHERS
    //> TIMERS
    /* Periodically publish status from the rosbag recorder node to the ros network. */
    rclcpp::TimerBase::SharedPtr status_timer_;
    //> SERVICES
    /* Enable or disable the node from the state-machine node. */
    rclcpp::Service<interfaces::srv::EnableNode>::SharedPtr enable_node_service_server_;
    //> ACTIONS
    //> DYNAMIC RECONFIGURE SERVER
    /* The handle used to dynamically reconfigure the ros parameters. */
    OnSetParametersCallbackHandle::SharedPtr dynamic_param_handle_;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> ROS FUNCTIONS
    //> MESSAGE POPULATERS
    //> SUBSCRIBERS CALLBACKS
    //> TIMERS CALLBACKS
    /* A function callback linked to status_timer_ used to publish data from the node.
       It wraps the processStatusTimerCb virtual function. */
    void wrapProcessStatusTimerCb();
    /* A function callback linked to status_timer_ used to publish the status of the node.
       Virtual, so needs to be defined in derived classes depending on the existing ros attributes. */
    virtual void processStatusTimerCb(const interfaces::msg::StatusCoreNode& _core_status_node_msg) = 0;
    //> SERVICES CALLBACKS
    /* The callback function processing the call to the service. */
    void enableNodeServiceCb(
      std::shared_ptr<interfaces::srv::EnableNode::Request> req,
         std::shared_ptr<interfaces::srv::EnableNode::Response> res);  // NOLINT
    //> ACTIONS CALLBACKS
    //> DYNAMIC RECONFIGURE SERVER CALLBACK
    rcl_interfaces::msg::SetParametersResult processParametersCallback(const std::vector<rclcpp::Parameter>& _parameters);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif  // _LIBRARY_ROS2NODE_HPP_
