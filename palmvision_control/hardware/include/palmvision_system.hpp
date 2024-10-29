#ifndef PALMVISION_CONTROL__PALMVISION_SYSTEM_HPP_
#define PALMVISION_CONTROL__PLAMVISION_SYSYTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "visibility_control.h"
#include "teensy_comms.hpp"
#include "wheel.hpp"
#include "servo.hpp"

namespace palmvision_control
{
class PalmVisionControlHardware : public hardware_interface::SystemInterface
{

struct Config
{
    std::string f_left_wheel_name = "";
    std::string b_left_wheel_name = "";
    std::string f_right_wheel_name = "";
    std::string b_right_wheel_name = "";
    std::string servo_name = "";
    std::string device = "";
    int baud_rate = 0;
    int timeout_ms = 0;
    int enc_counts_per_rev = 0;
};

public:
    RCLCPP_SHARED_PTR_DEFINITIONS(PalmVisionControlHardware);

    PALMVISION_CONTROL_PUBLIC
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    PALMVISION_CONTROL_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    PALMVISION_CONTROL_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    PALMVISION_CONTROL_PUBLIC
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    PALMVISION_CONTROL_PUBLIC
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;


    PALMVISION_CONTROL_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    PALMVISION_CONTROL_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    PALMVISION_CONTROL_PUBLIC
    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    PALMVISION_CONTROL_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;


private:

    TeensyComms comms_;
    Config cfg_;
    Wheel wheel_f_l_; //front left
    Wheel wheel_f_r_; //front right
    Wheel wheel_b_l_;  //back left
    Wheel wheel_b_r_;  //back right
    Servo servo;

};
}
#endif