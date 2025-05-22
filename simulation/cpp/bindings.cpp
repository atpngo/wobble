// bindings.cpp
#include <pybind11/pybind11.h>
#include "pid.h"

namespace py = pybind11;

PYBIND11_MODULE(control, m)
{
    py::class_<PID>(m, "PID")
        .def(py::init<double, double, double, double>())
        .def("reset", &PID::reset)
        .def("get_signal", &PID::get_signal);
}
