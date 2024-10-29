#ifndef PALMVISION_CONTROL_SERVO_HPP
#define PALMVISION_CONTROL_WHEEL_HPP

#include <string>
#include <cmath>

class Servo
{
public:
    std::string name = "";
    double cmd = 0; double pos = 0;

    Servo() = default;

    Servo(const std::string &servo_name){setup(servo_name);}

    void setup(const std::string &servo_name){name = servo_name;}

};

#endif