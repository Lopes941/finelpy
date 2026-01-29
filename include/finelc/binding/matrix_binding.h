#pragma once

#include <finelc/matrix.h>

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h> 
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <stdexcept>
#include <vector>
#include <memory>
namespace py = pybind11;

namespace finelc{

    Matrix from_python_to_matrix(py::object py_mat);

    py::object from_matrix_to_python(const Matrix& mat);
    
} // namespace finelc

namespace pybind11 { namespace detail {

    template <> struct type_caster<finelc::Matrix> {

        public:

            PYBIND11_TYPE_CASTER(finelc::Matrix, _("Matrix"));

            // Python -> C++
            bool load(handle src, bool){
                try {
                    value = finelc::from_python_to_matrix( py::reinterpret_borrow<py::object>(src));
                    return true;
                } catch (...) {
                    return false;
                }
            }

            // C++ -> Python
            static handle cast(const finelc::Matrix& mat, return_value_policy, handle) {
                return finelc::from_matrix_to_python(mat).release();
            }

    };

    
}} // namespace pybind11::detail
