#ifndef JOINT_CONTROL_INCLUDE_GUARD_HPP
#define JOINT_CONTROL_INCLUDE_GUARD_HPP

/// \file
/// \brief JointControl Library that converts Joystick commands to motion
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include "spot_interfaces/msg/joint_angles.hpp"

// TODO: interface to I2C
// TODO: interface to Servo controller

namespace joint_control
{

    // \brief JointControl class responsible for convertick Joystick commands into linear and angular velocity
    class JointControl : public rclcpp::Node
    {
    public:
        // \brief JointControl constructor that defines the axes used for control and sets their scaling factor
        JointControl();

        // \brief Takes a JointAngles message and writes the angles into the motors via I2C
        // \param angles: JointAngles describing what angle each motor should be at
	    void joint_callback(const spot_interfaces::msg::JointAngles::ConstPtr& joints);

    private:
        // subscribers
        rclcpp::Subscription<spot_interfaces::msg::JointAngles>::SharedPtr joint_sub_;

        // TODO: any items for motor config
        // joint to motor pin number
        // joint limits

        // timer and timekeeping
        rclcpp::Time current_time_;
        rclcpp::Time last_time_;
        rclcpp::TimerBase::SharedPtr timer_;
    };

}

#endif /* JOINT_CONTROL_INCLUDE_GUARD_HPP */