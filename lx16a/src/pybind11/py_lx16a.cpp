#include <pybind11/pybind11.h>

namespace py = pybind11;

void init_lx16a_time(py::module &m);
void init_lx16a_encoder_filter(py::module &m);

// Create a Python extension module called '_lx16a'.
PYBIND11_MODULE(_lx16a, m) {
    m.doc() = "pybind11 'lx16a' module";

    init_lx16a_time(m);
    init_lx16a_encoder_filter(m);
}
