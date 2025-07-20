/**********************************************************************************************************************/
//   author: nicolas.erbetti.k@gmail.com
//   brief: This file defines the header of the Ros2Parameter structure.
//          It is used to represent the structure of any ros parameter we might want to load to our ros 2 node.
//          It is a base structure.​
//   copyright: © 2025 Nicolas Erbetti. All rights reserved.
/**********************************************************************************************************************/

#ifndef _LIBRARY_PARAMETER_HPP_
#define _LIBRARY_PARAMETER_HPP_

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> DEPENDENCIES
//> C/C++ libraries
#include <optional>
#include <string>
#include <variant>
//> ROS dedicated libraries
#include "rclcpp/rclcpp.hpp"
//> Custom-made libraries
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> STRUCT DECLARATION
struct Ros2ParameterInterface {
    /* A struct to instantiate a Ros2ParameterInterface, which is used to store the parameter in a single unordered map. ​*/

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> CONSTRUCTORS
    /* Constructor of the struct. Declared as explicit to prevent implicit conversions. */
    explicit Ros2ParameterInterface(std::string _name) {
        // Initialize the name of the parameter.
        this->name_ = _name;
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> DESTRUCTORS
    /* Destructor of the struct.
       Must be declared virtual and defined if the struct is a base struct (to avoid memory leak when doing polymorphism).
       Or, must be declared pure virtual and defined if the struct is an interface (to prevent instanciation).
       In any case, must be declared inline to prevent its definition to be spread across multiple cpp file including it. */
    inline ~Ros2ParameterInterface() {
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> ACCESSORS
    //> GETTERS / All getters are declared as const since they are not modifying any class attributes.
    //> SETTERS / All setters return an int, 0 or -1 if the setting was successful or not, respectively.
    /* The setter of the variable through the ros parameter.
       Automatically routes to the correct typed setValue.
       Declared as purely virtual to allow its override from the typed derived class.
       Also, overload of the regular setValue() method. */
    virtual int setValue(const rclcpp::Parameter& _param) = 0;
    //////////////////////////////////////////////////////////////////////////////////////////////////////
   


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> C/C++ ATTRIBUTES
    //> DYNAMIC / They can change during runtime.
    /* The name of the ros parameter. Used to retrieve it from the yaml file. */
    std::string name_;
    //> FIXED / They cannot change during runtime. Declared as const.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
};

template <typename T>
struct Ros2Parameter : Ros2ParameterInterface {
    /* A struct to instantiate a Ros2Parameter. ​*/

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> CONSTRUCTORS
    /* Constructor of the struct. Declared as explicit to prevent implicit conversions. */
    explicit Ros2Parameter(std::string _name,
        std::optional<T> _min_value = std::nullopt, std::optional<T> _max_value = std::nullopt) : 
            Ros2ParameterInterface(_name) {
        // Initialize the value.
        this->value_ = new T();
        // Initialize the minimum value of the parameter, if they exist.
        this->min_value_ = _min_value;
        // Initialize the maximum value of the parameter, if they exist.
        this->max_value_ = _max_value;
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> DESTRUCTORS
    /* Destructor of the struct.
       Must be declared virtual and defined if the struct is a base struct (to avoid memory leak when doing polymorphism).
       Or, must be declared pure virtual and defined if the struct is an interface (to prevent instanciation).
       In any case, must be declared inline to prevent its definition to be spread across multiple cpp file including it. */
    inline ~Ros2Parameter() {
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> ACCESSORS
    //> GETTERS / All getters are declared as const since they are not modifying any class attributes.
    /* The getter of the variable.
       Automatically routes to the correctly typed value. 
       Declated as override as a safety, to specify that we are intending to override a virtual function. */
    T getValue() const;
    //> SETTERS / All setters return an int, 0 or -1 if the setting was successful or not, respectively.
    /* The setter of the variable. Optional, since the variable is public in the first place.
       However, this allows to make sure that a numeric value is within bounds. */
    int setValue(T _value);
    /* The setter of the variable through the ros parameter.
       Automatically routes to the correctly typed setValue.
       Declared as override as a safety, to specify that we are intending to override a virtual function.
       Also, overload of the regular setValue() method. */
    int setValue(const rclcpp::Parameter& _param) override;
    /* The setter of the variable through the ros parameter.
       Automatically routes to the correctly typed setValue.
       Declared as override as a safety, to specify that we are intending to override a virtual function.
       Also, overload of the regular setValue() method. */
    int setValuePtr(T* _value_ptr);
    //////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///> C/C++ ATTRIBUTES
    //> DYNAMIC / They can change during runtime.
    /* The value of the variable itself. */
    T* value_;
    /* The minimum value that the variable can take when it comes to numeric terms.
       Optional, should only exist if the value is numerical. */
    std::optional<T> min_value_;
    /* The maximum value that the variable can take when it comes to numeric terms.
       Optional, should only exist if the value is numerical. */
    std::optional<T> max_value_;
    //> FIXED / They cannot change during runtime. Declared as const.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///> ACCESSORS
//> GETTERS
template <typename T>
T Ros2Parameter<T>::getValue() const {
    // Return.
    return *this->value_;
}
//> SETTERS
template <typename T>
int Ros2Parameter<T>::setValue(T _value) {
    // Initialize return variable.
    int result = 0;
    std::cout << "Name " << this->name_ << std::endl;
    // If the value is numeric (int, float, double, or BOOL) and its bounds are defined...
    if constexpr (std::is_arithmetic_v<T>) {
        if (this->min_value_.has_value() && this->max_value_.has_value()) {
            // We check if it is within bounds...
            if (_value < this->min_value_.value() || _value > this->max_value_.value()) {
                // If not, we return an error.
                std::cout << "Invalid value. " << static_cast<double>(_value) << " needs be within [" << this->min_value_.value() << ", " << this->max_value_.value() << "]." << std::endl;
                result = -1;
            } else {
            // If yes, we update the registered value..
                *this->value_ = _value;
            }
        } else {
            // If there is no min or max value, we just update the registered value..
            *this->value_ = _value;
        }
    } else {
    // Otherwise, (for instance, for strings) we just update the registered value.
        *this->value_ = _value;
    }
    std::cout << "Variable set " << _value << std::endl;
    std::cout << "Value set " << *this->value_ << std::endl;
    // Return.
    return result;
}
template <typename T>
int Ros2Parameter<T>::setValue(const rclcpp::Parameter& _param) {
    // Initialize the return variable.
    int result = 0;
    // Try to set the value from the passed ros parameter.
    try {
        // Return 0 if everything was fine.
        result = setValue(_param.get_value<T>());
    } catch (std::exception& e) {
        // If there was an issue, return 1.
        std::cout << "Error for [" << _param.get_name() << "]: " << e.what() << std::endl;
        result = 1;
    }
    // Return.
    return result;
}
template <typename T>
int Ros2Parameter<T>::setValuePtr(T* _value_ptr) {
    // Initialize the return variable.
    int result = 0;
    // Try to set the value from the passed ros parameter.
    result = setValue(*_value_ptr);
    // Update the pointer.
    this->value_ = _value_ptr;
    // Return.
    return result;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif  // _LIBRARY_PARAMETER_HPP_
