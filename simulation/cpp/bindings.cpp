// bindings.cpp
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "pid.h"
#include "lqr.h"

namespace py = pybind11;

PYBIND11_MODULE(control, m)
{
    py::class_<PID>(m, "PID")
        .def(py::init<double, double, double, double>())
        .def("reset", &PID::reset)
        .def("get_signal", &PID::get_signal);

    py::class_<LQR>(m, "LQR")
        .def(py::init<float, float>())  // ctor(k_theta, k_theta_dot)
        .def("compute", &LQR::compute); // compute(theta_deg, theta_dot_deg)
}
