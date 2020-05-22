#include "curio_base/rover_base_hal.h"
#include <lx16a/lx16a_time.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <tuple>
#include <exception>

namespace py = pybind11;

// Helper class to redirect virtual calls back to Python.
class PyRoverBaseHAL : public curio_base::RoverBaseHAL
{
    // Destructor
    ~PyRoverBaseHAL() override {}

    // Inherit constructors
    using curio_base::RoverBaseHAL::RoverBaseHAL;

    size_t getNumWheels() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            size_t,
            RoverBaseHAL,
            "get_num_wheels",
            get_num_wheels
        );
    }

    size_t getNumSteers() const override
    {
        PYBIND11_OVERLOAD_PURE_NAME(
            size_t,
            RoverBaseHAL,
            "get_num_steers",
            get_num_steers
        );
    }

    double getWheelPosition(const ros::Time &ros_time, int i) const override
    {
        // Convert ros::Time to lx16a::Time for interop.
        lx16a::Time time(ros_time.sec, ros_time.nsec);

        pybind11::gil_scoped_acquire gil;
        pybind11::function overload = pybind11::get_overload(this, "get_wheel_position");
        if (overload)
        {
            auto obj = overload(time, i);
            return py::cast<double>(obj);
        }
        throw std::runtime_error("Tried to call pure virtual function \"RoverBaseHAL::getWheelPosition\"");
    }

    double getWheelVelocity(const ros::Time &ros_time, int i) const override
    {
        lx16a::Time time(ros_time.sec, ros_time.nsec);

        pybind11::gil_scoped_acquire gil;
        pybind11::function overload = pybind11::get_overload(this, "get_wheel_velocity");
        if (overload)
        {
            auto obj = overload(time, i);
            return py::cast<double>(obj);
        }
        throw std::runtime_error("Tried to call pure virtual function \"RoverBaseHAL::getWheelVelocity\"");
    }

    void setWheelVelocity(const ros::Time &ros_time, int i, double velocity) override
    {
        lx16a::Time time(ros_time.sec, ros_time.nsec);

        pybind11::gil_scoped_acquire gil;
        pybind11::function overload = pybind11::get_overload(this, "set_wheel_velocity");
        if (overload)
        {
            auto obj = overload(time, i, velocity);
            return;
        }
        throw std::runtime_error("Tried to call pure virtual function \"RoverBaseHAL::setWheelVelocity\"");
    }

    double getSteerAngle(const ros::Time &ros_time, int i) const override
    {
        lx16a::Time time(ros_time.sec, ros_time.nsec);

        pybind11::gil_scoped_acquire gil;
        pybind11::function overload = pybind11::get_overload(this, "get_steer_angle");
        if (overload)
        {
            auto obj = overload(time, i);
            return py::cast<double>(obj);
        }
        throw std::runtime_error("Tried to call pure virtual function \"RoverBaseHAL::getSteerAngle\"");
    }

    void setSteerAngle(const ros::Time &ros_time, int i, double angle) override
    {
        lx16a::Time time(ros_time.sec, ros_time.nsec);

        pybind11::gil_scoped_acquire gil;
        pybind11::function overload = pybind11::get_overload(this, "set_steer_angle");
        if (overload)
        {
            auto obj = overload(time, i, angle);
            return;
        }
        throw std::runtime_error("Tried to call pure virtual function \"RoverBaseHAL::setSteerAngle\"");
    }

    void getWheelPositions(const ros::Time &ros_time, std::vector<double>& positions) const override
    {        
        auto k_num_wheels = getNumWheels();
        for (auto i = 0; i<k_num_wheels; ++i)
        {
            positions[i] = getWheelPosition(ros_time, i);
        }   
    }

    void getWheelVelocities(const ros::Time &ros_time, std::vector<double>& velocities) const override
    {
        auto k_num_wheels = getNumWheels();
        for (auto i = 0; i<k_num_wheels; ++i)
        {
            velocities[i] = getWheelVelocity(ros_time, i);
        }   
    }

    void setWheelVelocities(const ros::Time &ros_time, const std::vector<double>& velocities) override
    {
        auto k_num_wheels = getNumWheels();
        for (auto i = 0; i<k_num_wheels; ++i)
        {
            setWheelVelocity(ros_time, i, velocities[i]);
        }   
    }

    void getSteerAngles(const ros::Time &ros_time, std::vector<double>& positions) const override
    {
        auto k_num_steers = getNumSteers();
        for (auto i = 0; i<k_num_steers; ++i)
        {
            positions[i] = getSteerAngle(ros_time, i);
        }   
    }

    void setSteerAngles(const ros::Time &ros_time, const std::vector<double>& positions) override
    {
        auto k_num_steers = getNumSteers();
        for (auto i = 0; i<k_num_steers; ++i)
        {
            setSteerAngle(ros_time, i, positions[i]);
        }   
    }
};

void init_rover_base_hal(py::module &m)
{
    py::class_<curio_base::RoverBaseHAL, PyRoverBaseHAL>(m, "RoverBaseHAL")
        .def(py::init<>())
        .def("get_num_wheels", &curio_base::RoverBaseHAL::getNumWheels, "Get the number of wheels")
        .def("get_num_steers", &curio_base::RoverBaseHAL::getNumSteers, "Get the number of steers")
        .def("get_wheel_position", &curio_base::RoverBaseHAL::getWheelPosition, "Get the angular position of the i-th wheel [rad]", py::arg("time"), py::arg("i"))
        .def("get_wheel_velocity", &curio_base::RoverBaseHAL::getWheelVelocity, "Get the angular velocity of the i-th wheel [rad/s]", py::arg("time"), py::arg("i"))
        .def("set_wheel_velocity", &curio_base::RoverBaseHAL::setWheelVelocity, "Get the angular velocity of the i-th wheel [rad/s]", py::arg("time"), py::arg("i"), py::arg("velocity"))
        .def("get_steer_angle", &curio_base::RoverBaseHAL::getSteerAngle, "Get the steering angle of the i-th steer [rad]", py::arg("time"), py::arg("i"))
        .def("set_steer_angle", &curio_base::RoverBaseHAL::setSteerAngle, "Set the angle of the i-th steer [rad]", py::arg("time"), py::arg("i"), py::arg("angle"))
        ;
}
