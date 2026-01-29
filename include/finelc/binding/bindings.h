#pragma once

#include <pybind11/pybind11.h>

namespace py = pybind11;

// Declare binding functions
void bind_geometry(py::module_& handle);
void bind_material(py::module_& handle);
void bind_mesh(py::module_& handle);
void bind_element(py::module_& handle);
void bind_analysis(py::module_& handle);
void bind_solver(py::module_& handle);
