#ifndef PALMVISION_CONTROL_TEENSY_COMMS_HPP
#define PALMVISION_CONTROL_TEENSY_COMMS_HPP

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class TeensyComms
{
public:

    TeensyComms() = default;

    void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
    {
        timeout_ms_ = timeout_ms;
        serial_conn_.Open(serial_device);
        serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
    }

    void disconnect(){serial_conn_.Close();}

    bool connected() const{return serial_conn_.IsOpen();}

    std::string send_msg(const std::string &msg_to_send, bool print_output = false)
    {
        serial_conn_.FlushIOBuffers();
        serial_conn_.Write(msg_to_send);

        std::string response = "";
        try
        {
            // Responses end with \r\n so we will read up to (and including) the \n.
            serial_conn_.ReadLine(response, '\n', timeout_ms_);
        }
        catch (const LibSerial::ReadTimeout&)
        {
            // std::cerr << "The ReadByte() call has timed out." << std::endl ;
        }

        if (print_output)
        {
            std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
        }
        
        return response;
    }

    void send_empty_msg(){std::string response = send_msg("\r");}

    void reset_encoder_values(){std::string response = send_msg("r\r");}

    // REFACTOR IT TO 4 ENCODER DATA, MOTOR COMMAND, AND INCLUDE SERVO
    void read_encoder_values(int &val_1, int &val_2, int &val_3, int &val_4, double &val_5) 
    {
        // Send the request and get the response
        std::string response = send_msg("e\n");

        // Check if the response starts with 'e' and has the expected format
        if (response.substr(0, 2) != "e|") {
            // Handle error: unexpected response format
            return;
        }

        std::string delimiter = "|";
        size_t pos = 0;

        // Move past the 'e'
        response = response.substr(2); // Skip 'e|'

        // Parse the first value (fl)
        pos = response.find(delimiter);
        std::string token = response.substr(0, pos);
        val_1 = std::atoi(token.substr(token.find(':') + 1).c_str());
        response.erase(0, pos + delimiter.length());

        // Parse the second value (fr)
        pos = response.find(delimiter);
        token = response.substr(0, pos);
        val_2 = std::atoi(token.substr(token.find(':') + 1).c_str());
        response.erase(0, pos + delimiter.length());

        // Parse the third value (rl)
        pos = response.find(delimiter);
        token = response.substr(0, pos);
        val_3 = std::atoi(token.substr(token.find(':') + 1).c_str());
        response.erase(0, pos + delimiter.length());

        // Parse the fourth value (rr)
        pos = response.find(delimiter);
        token = response.substr(0, pos);
        val_4 = std::atoi(token.substr(token.find(':') + 1).c_str());
        response.erase(0, pos + delimiter.length());

        // Parse the fifth value (s)
        val_5 = std::atof(response.substr(response.find(':') + 1).c_str());
    }

    void set_motor_values(double val_1, double val_2, double val_3, double val_4, double val_5) 
    {
        // Front Left, Front Right, Back Left, Back Right
        std::stringstream ss;
        ss << "m|fl:" << val_1 
        << "|fr:" << val_2 
        << "|rl:" << val_3 
        << "|rr:" << val_4 
        << "|s:" << val_5 << "\r";
        send_msg(ss.str());
    }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif