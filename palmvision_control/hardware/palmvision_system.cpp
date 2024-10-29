#include "include/palmvision_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace palmvision_control
{

hardware_interface::CallbackReturn PalmVisionControlHardware::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    cfg_.f_left_wheel_name = info_.hardware_parameters["front_left_wheel_name"];
    cfg_.f_right_wheel_name = info_.hardware_parameters["front_right_wheel_name"];
    cfg_.b_left_wheel_name = info_.hardware_parameters["back_left_wheel_name"];
    cfg_.b_right_wheel_name = info_.hardware_parameters["back_right_wheel_name"];
    cfg_.servo_name = info_.hardware_parameters["servo_name"];
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

    wheel_f_l_.setup(cfg_.f_left_wheel_name, cfg_.enc_counts_per_rev);
    wheel_f_r_.setup(cfg_.f_right_wheel_name, cfg_.enc_counts_per_rev);
    wheel_b_l_.setup(cfg_.b_left_wheel_name, cfg_.enc_counts_per_rev);
    wheel_b_r_.setup(cfg_.b_right_wheel_name, cfg_.enc_counts_per_rev);
    
    servo.setup(cfg_.servo_name);

    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION))
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("PalmVisionControlHardware"),
                "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
                hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION))
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("PalmVisionControlHardware"),
                "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
                hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION);
            return hardware_interface::CallbackReturn::ERROR;
        }

    }

    return hardware_interface::CallbackReturn::SUCCESS;

}

std::vector<hardware_interface::StateInterface> PalmVisionControlHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_f_l_.name, hardware_interface::HW_IF_POSITION, &wheel_f_l_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_f_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_f_l_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_b_l_.name, hardware_interface::HW_IF_POSITION, &wheel_b_l_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_b_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_b_l_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_f_r_.name, hardware_interface::HW_IF_POSITION, &wheel_f_r_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_f_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_f_r_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_b_r_.name, hardware_interface::HW_IF_POSITION, &wheel_b_r_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_b_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_b_r_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        servo.name, hardware_interface::HW_IF_POSITION, &servo.pos));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> PalmVisionControlHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_f_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_f_l_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_b_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_b_l_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_f_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_f_r_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_b_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_b_r_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        servo.name, hardware_interface::HW_IF_POSITION, &servo.pos));

    return command_interfaces;

}

hardware_interface::CallbackReturn PalmVisionControlHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("PalmVisionControlHardware"), "Configuring ...please wait...");
    comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
    RCLCPP_INFO(rclcpp::get_logger("PalmVisionControlHardware"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PalmVisionControlHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("PalmVisionControlHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("PalmVisionControlHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PalmVisionControlHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("PalmVisionControlHardware"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  else{comms_.reset_encoder_values();}
  RCLCPP_INFO(rclcpp::get_logger("PalmVisionControlHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PalmVisionControlHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("PalmVisionControlHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("PalmVisionControlHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type PalmVisionControlHardware::read(
    const rclcpp::Time &, const rclcpp::Duration & period
)
{
    if (!comms_.connected())
    {
        return hardware_interface::return_type::ERROR;
    }

    comms_.read_encoder_values(wheel_f_l_.enc, wheel_f_r_.enc, wheel_b_l_.enc, wheel_b_r_.enc, servo.pos);

    double delta_seconds = period.seconds();

    double pos_prev = wheel_f_l_.pos;
    wheel_f_l_.pos = wheel_f_l_.cal_enc_angle();
    wheel_f_l_.vel = (wheel_f_l_.pos - pos_prev)/ delta_seconds;

    pos_prev = wheel_b_l_.pos;
    wheel_b_l_.pos = wheel_b_l_.cal_enc_angle();
    wheel_b_l_.vel = (wheel_b_l_.pos - pos_prev)/ delta_seconds;

    pos_prev = wheel_f_r_.pos;
    wheel_f_r_.pos = wheel_f_r_.cal_enc_angle();
    wheel_f_r_.vel = (wheel_f_r_.pos - pos_prev)/ delta_seconds;

    pos_prev = wheel_b_r_.pos;
    wheel_b_r_.pos = wheel_b_r_.cal_enc_angle();
    wheel_b_r_.vel = (wheel_b_r_.pos - pos_prev)/ delta_seconds;

    return hardware_interface::return_type::OK;

}

hardware_interface::return_type palmvision_control::PalmVisionControlHardware::write(
    const rclcpp::Time &, const rclcpp::Duration &period
)
{
    if (!comms_.connected())
    {
        return hardware_interface::return_type::ERROR;
    }

    // RCLCPP_INFO(
    //     rclcpp::get_logger("PalmVisionControlHardware"),
    //     "Commands - FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f, Servo: %.2f",
    //     wheel_f_l_.cmd, wheel_f_r_.cmd, 
    //     wheel_b_l_.cmd, wheel_b_r_.cmd,
    //     servo.cmd
    // );

    double radius = 0.127;
    double circumference = 2 * M_PI * radius; // Circumference of the circle

    float motor_f_l_rpm = (wheel_f_l_.cmd/ circumference) * 60;
    float motor_b_l_rpm = (wheel_b_l_.cmd/ circumference) * 60;
    float motor_f_r_rpm = (wheel_f_r_.cmd/ circumference) * 60;
    float motor_b_r_rpm = (wheel_b_r_.cmd/ circumference) * 60;

    comms_.set_motor_values(
        motor_f_l_rpm, motor_f_r_rpm, motor_b_l_rpm, motor_b_r_rpm, servo.cmd
    );

    return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    palmvision_control::PalmVisionControlHardware, hardware_interface::SystemInterface
)