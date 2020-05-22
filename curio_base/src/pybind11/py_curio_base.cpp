#include <pybind11/pybind11.h>

namespace py = pybind11;

void init_time(py::module &m);
void init_rover_base_hal(py::module &m);

// Create a Python extension module called '_curio_base'.
PYBIND11_MODULE(_curio_base, m) {
    m.doc() = "pybind11 'curio_base' module";

    init_time(m);
    init_rover_base_hal(m);
}
