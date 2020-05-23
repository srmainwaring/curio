#include "lx16a/lx16a_driver.h"
#include "lx16a/lx16a_exception.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <tuple>

namespace py = pybind11;
void init_lx16a_driver(py::module &m)
{
    py::class_<lx16a::LX16ADriver>(m, "LX16ADriver")
        .def(py::init<>())
        .def("open", &lx16a::LX16ADriver::open, "Open the serial port for communication")        
        .def("close", &lx16a::LX16ADriver::close, "Close the serial port immediately")        
        .def("is_open", &lx16a::LX16ADriver::isOpen, "Get the open status of the serial port")        
        .def("get_port", &lx16a::LX16ADriver::getPort, "Get the serial port identifier")    
        .def("set_port", &lx16a::LX16ADriver::setPort, "Set the serial port identifier", py::arg("port"))        
        .def("get_baudrate", &lx16a::LX16ADriver::getBaudrate, "Get the baudrate for the serial port")        
        .def("set_baudrate", &lx16a::LX16ADriver::setBaudrate, "Set the baudrate for the serial port", py::arg("baudrate"))        
        .def("set_timeout", [](lx16a::LX16ADriver &driver, int timeout)
        {
            serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(timeout);
            driver.setTimeout(serial_timeout);
        }, "Set the serial timeout [ms].", py::arg("timeout"))  
        .def("set_response_timeout", [](lx16a::LX16ADriver &driver, int timeout)
        {
            std::chrono::milliseconds timeout_millis(timeout);
            driver.setResponseTimeout(timeout_millis);
        }, "Set the serial response timeout [ms].", py::arg("timeout"))  
        .def("get_response_timeout", [](lx16a::LX16ADriver &driver)
        {
            auto timeout = driver.getResponseTimeout();
            return timeout.count();
        }, "Get the serial response timeout [ms].")  
        .def("move", &lx16a::LX16ADriver::move, "Move a servo to the commanded position in a given time", py::arg("servo_id"), py::arg("position"), py::arg("move_time") = 0)
        .def("get_move", [](lx16a::LX16ADriver& driver, uint8_t servo_id)
        {
            int16_t position, move_time;
            driver.getMove(servo_id, position, move_time);
            return std::tuple<int16_t, int16_t>(position, move_time);
        }, "Read the current commanded position and move time", py::arg("servo_id"))
        .def("set_prepared_move", &lx16a::LX16ADriver::setPreparedMove, "Prepare a servo for a move to a new position", py::arg("servo_id"), py::arg("position"), py::arg("move_time"))
        .def("get_prepared_move", &lx16a::LX16ADriver::getPreparedMove, "Get the move time of a prepared servo move", py::arg("servo_id"))
        .def("move_start", &lx16a::LX16ADriver::moveStart, "Start a prepared servo move", py::arg("servo_id"))        
        .def("move_stop", &lx16a::LX16ADriver::moveStop, "Immediately stop the servo from moving", py::arg("servo_id"))        
        .def("set_position_offset", &lx16a::LX16ADriver::setPositionOffset, "Set the servo position offset", py::arg("servo_id"), py::arg("position_offset"))        
        .def("save_position_offset", &lx16a::LX16ADriver::savePositionOffset, "", py::arg("servo_id"))
        .def("get_position_offset", &lx16a::LX16ADriver::getPositionOffset, "", py::arg("servo_id"))
        .def("set_position_limits", &lx16a::LX16ADriver::setPositionLimits, "Set the servo position minimum and maximum limits", py::arg("servo_id"), py::arg("min_position"), py::arg("max_position"))
        .def("get_position_limits", [](lx16a::LX16ADriver& driver, uint8_t servo_id)
        {
            uint16_t min_position, max_position;
            driver.getPositionLimits(servo_id, min_position, max_position);
            return std::tuple<uint16_t, uint16_t>(min_position, max_position);
        }, "Get the servo position minimum and maximum limits", py::arg("servo_id"))
        .def("set_voltage_limits", &lx16a::LX16ADriver::setVoltageLimits, "Set the servo voltage minimum and maximum limits", py::arg("servo_id"), py::arg("min_voltage"), py::arg("max_voltage"))
        .def("get_voltage_limits", [](lx16a::LX16ADriver& driver, uint8_t servo_id)
        {
            double min_voltage, max_voltage;
            driver.getVoltageLimits(servo_id, min_voltage, max_voltage);
            return std::tuple<double, double>(min_voltage, max_voltage);
        }, "Get the servo voltage minimum and maximum limits", py::arg("servo_id"))
        .def("set_max_temperature_limit", &lx16a::LX16ADriver::setMaxTemperatureLimit, "Set the servo maximum temperature limit", py::arg("servo_id"), py::arg("max_temperature"))
        .def("get_max_temperature_limit", &lx16a::LX16ADriver::getMaxTemperatureLimit, "", py::arg("servo_id"))
        .def("get_temperature", &lx16a::LX16ADriver::getTemperature, "Get the servo temperature", py::arg("servo_id"))
        .def("get_voltage", &lx16a::LX16ADriver::getVoltage, "Get the servo voltage", py::arg("servo_id"))
        .def("get_position", &lx16a::LX16ADriver::getPosition, "Get the servo position", py::arg("servo_id"))
        .def("set_motor_mode", &lx16a::LX16ADriver::setMotorMode, "Set the servo to 'motor' mode", py::arg("servo_id"), py::arg("duty"))
        .def("set_servo_mode", &lx16a::LX16ADriver::setServoMode, "Set the servo to 'servo' mode", py::arg("servo_id"))
        .def("get_mode", [] (lx16a::LX16ADriver &driver, uint8_t servo_id)
        {
            uint8_t mode;
            int16_t duty;
            driver.getMode(servo_id, mode, duty);
            return std::tuple<uint8_t, int16_t>(mode, duty);
        }, "Get the servo mode", py::arg("servo_id"))
        .def("set_motor_on", &lx16a::LX16ADriver::setMotorOn, "Power the servo motor on", py::arg("servo_id"))
        .def("set_motor_off", &lx16a::LX16ADriver::setMotorOff, "Power the servo motor off", py::arg("servo_id"))
        .def("is_motor_on", &lx16a::LX16ADriver::isMotorOn, "", py::arg("servo_id"))
        .def("set_led_on", &lx16a::LX16ADriver::setLedOn, "Turn the servo LED on", py::arg("servo_id"))
        .def("set_led_off", &lx16a::LX16ADriver::setLedOff, "Turn the servo LED off", py::arg("servo_id"))
        .def("is_led_on", &lx16a::LX16ADriver::isLedOn, "Get the state of the servo LED", py::arg("servo_id"))        
        .def("set_led_errors", &lx16a::LX16ADriver::setLedErrors, "Set the list of faults that cause the LED to flash", py::arg("servo_id"), py::arg("fault_code"))
        .def("get_led_errors", &lx16a::LX16ADriver::getLedErrors, "Get the list of faults that cause the LED to flash", py::arg("servo_id"))
        ;
}
