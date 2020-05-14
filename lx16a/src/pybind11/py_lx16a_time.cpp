#include "lx16a/lx16a_time.h"
#include <pybind11/pybind11.h>
#include <sstream>

namespace py = pybind11;

void init_lx16a_time(py::module &m)
{
    py::class_<lx16a::Time>(m, "Time")
        .def(py::init<>())
        .def_readwrite("sec", &lx16a::Time::sec)
        .def_readwrite("nsec", &lx16a::Time::nsec)
        .def("__repr__",
            [](const lx16a::Time &t)
            {
                std::stringstream ss;
                ss << "<lx16a.Time sec: " << t.sec
                << ", nsec: "<< t.nsec << ">";
                return ss.str();
            }
        );
}
