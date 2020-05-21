#include "lx16a/lx16a_encoder_filter.h"
#include "lx16a/lx16a_time.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <tuple>

namespace py = pybind11;

// Helper class to redirect virtual calls back to Python.
class PyLX16AEncoderFilter : public lx16a::LX16AEncoderFilter
{
    // Inherit constructors
    using lx16a::LX16AEncoderFilter::LX16AEncoderFilter;

    void init() override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            void,
            LX16AEncoderFilter,
            "init",
            init
        );
    }

    void add(uint8_t servo_id) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            void,
            LX16AEncoderFilter,
            "add",
            add,
            servo_id
        );
    }

    void update(uint8_t servo_id, const ros::Time &ros_time, int16_t duty, int16_t position) override
    {
        // Convert ros::Time to lx16a::Time for interop.
        lx16a::Time time(ros_time.sec, ros_time.nsec);

        pybind11::gil_scoped_acquire gil;
        pybind11::function overload = pybind11::get_overload(this, "update");
        if (overload)
        {
            auto obj = overload(servo_id, time, duty, position); 
        }
    }

    int16_t getRevolutions(uint8_t servo_id) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            int16_t,
            LX16AEncoderFilter,
            "get_revolutions",
            getRevolutions,
            servo_id
        );
    }

    int16_t getCount(uint8_t servo_id) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            int16_t,
            LX16AEncoderFilter,
            "get_count",
            getCount,
            servo_id
        );
    }

    int16_t getDuty(uint8_t servo_id) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            int16_t,
            LX16AEncoderFilter,
            "get_duty",
            getDuty,
            servo_id
        );
    }

    double getAngularPosition(uint8_t servo_id) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            double,
            LX16AEncoderFilter,
            "get_angular_position",
            getAngularPosition,
            servo_id
        );
    }

    // The Python overload returns a tuple (position, is_valid) as int16_t and
    // bool map to immutable Python types and arguments passed by reference
    // will not be modified.
    void getServoPosition(uint8_t servo_id, int16_t &position, bool &is_valid, bool map_position=true) const override
    {
        pybind11::gil_scoped_acquire gil;
        pybind11::function overload = pybind11::get_overload(this, "get_servo_position");
        if (overload)
        {
            auto obj = overload(servo_id, map_position); 
            if (py::isinstance<py::tuple>(obj))
            {
                // Cast the py::tuple to the STL type and get entries.
                auto value = obj.cast<std::tuple<int16_t, bool>>();
                position = std::get<0>(value);
                is_valid = std::get<1>(value);
                return;
            }
        }
    }

    int16_t getInvert(uint8_t servo_id) const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            int16_t,
            LX16AEncoderFilter,
            "get_invert",
            getInvert,
            servo_id
        );
    }

    void setInvert(uint8_t servo_id, bool is_inverted) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            void,
            LX16AEncoderFilter,
            "set_invert",
            setInvert,
            servo_id, is_inverted
        );
    }

    void reset(uint8_t servo_id, int16_t position) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            void,
            LX16AEncoderFilter,
            "reset",
            reset,
            servo_id, position
        );
    }

    void add_v(const std::vector<uint8_t> &servo_ids) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            void,
            LX16AEncoderFilter,
            "add_v",
            add_v,
            servo_ids
        );
    }

    // The Python overload returns a list (not efficient as copying in/out)
    void update_v(
        const std::vector<uint8_t> &servo_ids,
        const ros::Time &ros_time,
        const std::vector<int16_t> &duties,
        const std::vector<int16_t> &positions,
        std::vector<double> &angular_positions) override
    {
        // Convert ros::Time to lx16a::Time for interop.
        lx16a::Time time(ros_time.sec, ros_time.nsec);

        pybind11::gil_scoped_acquire gil;
        pybind11::function overload = pybind11::get_overload(this, "update_v");
        if (overload)
        {
            auto obj = overload(servo_ids, time, duties, positions); 
            if (py::isinstance<py::list>(obj))
            {
                // Cast the py::list to the STL type and get entries.
                auto value = obj.cast<std::vector<double>>();
                std::copy(value.begin(), value.end(), angular_positions.begin());
            }
        }
    }
};

void init_lx16a_encoder_filter(py::module &m)
{
    py::class_<lx16a::LX16AEncoderFilter, PyLX16AEncoderFilter>(m, "LX16AEncoderFilter")
        .def(py::init<>())
        .def("init", &lx16a::LX16AEncoderFilter::init, "Initialise the encoder filter")        
        .def("add", &lx16a::LX16AEncoderFilter::add, "Add a servo", py::arg("servo_id"))
        .def("update", &lx16a::LX16AEncoderFilter::update, "Update the encoder filter", py::arg("servo_id"), py::arg("ros_time"), py::arg("duty"),  py::arg("position"))
        .def("get_revolutions", &lx16a::LX16AEncoderFilter::getRevolutions, "Get the number of revoutions since reset", py::arg("servo_id"))
        .def("get_count", &lx16a::LX16AEncoderFilter::getCount, "Get the current encoder count since reset (filtered)", py::arg("servo_id"))
        .def("get_duty", &lx16a::LX16AEncoderFilter::getDuty, "Get the current encoder duty", py::arg("servo_id"))
        .def("get_angular_position", &lx16a::LX16AEncoderFilter::getAngularPosition, "Get the angular position of the encoder (filtered)", py::arg("servo_id"))
        .def("get_servo_position", &lx16a::LX16AEncoderFilter::getServoPosition, "Get the current (un-filtered) servo position", py::arg("servo_id"), py::arg("position"), py::arg("is_valid"),  py::arg("map_position") = true)
        .def("get_invert", &lx16a::LX16AEncoderFilter::getInvert, "Get the invert state: whether the encoder count is inverted", py::arg("servo_id"))
        .def("set_invert", &lx16a::LX16AEncoderFilter::setInvert, "Invert the direction of the encoder count", py::arg("servo_id"), py::arg("is_inverted"))
        .def("reset", &lx16a::LX16AEncoderFilter::reset, "Reset the encoder counters to zero", py::arg("servo_id"), py::arg("position"))
        .def("add_v", &lx16a::LX16AEncoderFilter::add_v, "Add servos", py::arg("servo_ids"))
        .def("update_v", &lx16a::LX16AEncoderFilter::update_v, "Update the encoder filter", py::arg("servo_ids"), py::arg("ros_time"), py::arg("duties"), py::arg("positions"), py::arg("angular_positions"))
        ;
}
