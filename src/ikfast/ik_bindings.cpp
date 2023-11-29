#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include "ikfast_interface.hpp"
#include "config.h"

// Bind the function to Python using pybind11
void init_core(py::module m)
{
    // py::module& core(m);
    m.doc() = "pyikfast"; // Optional module docstring

    m.def("IKFast_trans3D", py::overload_cast<const std::vector<double>>(&IKFast_trans3D), py::arg("trans"), "Compute IK for 3D translation");
}

PYBIND11_MODULE(PYBIND_NAME, m)
{
    init_core(m);
}
