#pragma once

#include <finelc/enumerations.h>
#include <finelc/matrix.h>

#include <finelc/material/material.h>
#include <finelc/material/constitutive.h>

#include <finelc/binding/matrix_binding.h>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h> 
#include <pybind11/stl.h> 

#include <vector>
#include <memory>

using namespace finelc;
namespace py = pybind11;

class ConstitutiveModelTrampoline: public IConstitutiveModel{

    public:

        ~ConstitutiveModelTrampoline() override =default;

        double get_property(const MaterialProperties& prop) override{
            PYBIND11_OVERLOAD_PURE(
                double,
                IConstitutiveModel,
                get_property,
                prop
            );
        }

        bool is_linear() const override{
            PYBIND11_OVERLOAD_PURE(
                bool,
                IConstitutiveModel,
                is_linear
            );
        }

        ConstitutiveType constitutive_model() const override{
            PYBIND11_OVERLOAD_PURE(
                ConstitutiveType,
                IConstitutiveModel,
                constitutive_model
            );
        }

        Matrix D(const Vector& loc, const Vector& ue) const override{
            PYBIND11_OVERLOAD_PURE(
                Matrix,
                IConstitutiveModel,
                D,
                loc, ue
            );
        }

};
