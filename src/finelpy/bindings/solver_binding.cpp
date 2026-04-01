#include <finelc/matrix.h>
#include <finelc/enumerations.h>

#include <finelc/analysis/analysis.h>
#include <finelc/solver/solver.h>
#include <finelc/result/result.h>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h> 
#include <pybind11/stl.h>

#include <finelc/binding/bindings.h>
#include <finelc/binding/matrix_binding.h>
#include <finelc/binding/geometry_binding.h>


#include <stdexcept>
#include <vector>
#include <memory>

using namespace finelc;
namespace py = pybind11;


void bind_solver(py::module_& handle){

    {
    py::enum_<SolverType>(
        handle, 
        "SolverType",
        R"pbdoc(
        Represents the solver method to be used.
        )pbdoc")
        .value("Direct", SolverType::Direct, R"pbdoc(Direct solver: Cholesky, LU)pbdoc")
        .value("Iterative", SolverType::Iterative, R"pbdoc(Iterative solver)pbdoc")
        ;
    }

    {
    py::enum_<ResultData>(
        handle, 
        "ResultData",
        R"pbdoc(
        Represents the results that can be reconstructed.
        )pbdoc")
        .value("UX", ResultData::UX, R"pbdoc(Displacement on X direction)pbdoc")
        .value("UY", ResultData::UY, R"pbdoc(Displacement on Y direction)pbdoc")
        .value("UZ", ResultData::UZ, R"pbdoc(Displacement on Z direction)pbdoc")
        .value("ABS_U", ResultData::ABS_U, R"pbdoc(Absolute value of displacement)pbdoc")

        .value("THETAX", ResultData::THETAX, R"pbdoc(Rotation on X direction)pbdoc")
        .value("THETAY", ResultData::THETAY, R"pbdoc(Rotation on Y direction)pbdoc")
        .value("THETAZ", ResultData::THETAZ, R"pbdoc(Rotation on Z direction)pbdoc")

        
        .value("EPSILON_XX", ResultData::EPSILON_XX, R"pbdoc(Normal strain XX)pbdoc")
        .value("EPSILON_YY", ResultData::EPSILON_YY, R"pbdoc(Normal strain YY)pbdoc")
        .value("EPSILON_ZZ", ResultData::EPSILON_ZZ, R"pbdoc(Normal strain ZZ)pbdoc")
        .value("EPSILON_XY", ResultData::EPSILON_XY, R"pbdoc(Shear strain XY)pbdoc")
        .value("EPSILON_XZ", ResultData::EPSILON_XZ, R"pbdoc(Shear strain XZ)pbdoc")
        .value("EPSILON_YZ", ResultData::EPSILON_YZ, R"pbdoc(Shear strain YZ)pbdoc")

        .value("SIGMA_XX", ResultData::SIGMA_XX, R"pbdoc(Normal stress XX)pbdoc")
        .value("SIGMA_YY", ResultData::SIGMA_YY, R"pbdoc(Normal stress YY)pbdoc")
        .value("SIGMA_ZZ", ResultData::SIGMA_ZZ, R"pbdoc(Normal stress ZZ)pbdoc")
        .value("SIGMA_XY", ResultData::SIGMA_XY, R"pbdoc(Shear stress XY)pbdoc")
        .value("SIGMA_XZ", ResultData::SIGMA_XZ, R"pbdoc(Shear stress XZ)pbdoc")
        .value("SIGMA_YZ", ResultData::SIGMA_YZ, R"pbdoc(Shear stress YZ)pbdoc")

        .value("SIGMA_VONMISES", ResultData::SIGMA_VONMISES, R"pbdoc(Equivalent von Mises stress)pbdoc")

        .value("NX", ResultData::NX, R"pbdoc(Normal force)pbdoc")
        .value("VY", ResultData::VY, R"pbdoc(Shear force)pbdoc")
        .value("MZ", ResultData::MZ, R"pbdoc(Bending moment)pbdoc")
        ;
    }

    {
    py::class_<StaticResult, std::shared_ptr<StaticResult>>(
        handle, 
        "StaticResult", 
        py::dynamic_attr(),
        R"pbdoc(
        Result object from Static Solver.
        )pbdoc")

        .def_property_readonly("u",
            &StaticResult::u,
            R"pbdoc(
            Global displacement vector.
            )pbdoc")

        .def_property_readonly("nodes",
            &StaticResult::nodes,
            R"pbdoc(
            Nodes from mesh.
            )pbdoc")

        .def_property_readonly("elements",
            &StaticResult::elements,
            R"pbdoc(
            Elements from mesh.
            )pbdoc")

        .def_property_readonly("analysis",
            &StaticResult::get_analysis,
            R"pbdoc(
            Analysis object.
            )pbdoc")


        .def("get_points", 
            &StaticResult::get_points,
            py::arg("internal_pts")=10,
            R"pbdoc(
            Get a grid on the domain with given number of internal points in elements.

            Parameters
            ------------
            internal_pts: int, optional (default=10)
                Number of internal points per element.

            Returns
            -----------
            grid_points: numpy.array
                Grid of points in global coordinates
            )pbdoc")

        .def("grid_data", 
            &StaticResult::grid_data,
            py::arg("result_id"),
            py::arg("internal_pts")=10,
            R"pbdoc(
            Get data in points from a grid in domain.

            Parameters
            ------------
            result_id: ResultData
                ID of data to be plotted.
            internal_pts: int, optional (default=10)
                Number of internal points per element.

            Returns
            -----------
            GridData: numpy.ndarray
                Vector containing tuples with point and value.
            )pbdoc")

        .def("get_max", 
            [](const StaticResult& self, ResultData id, int internal_pts) -> py::tuple {
                MaxMinResult res = self.get_max(id,internal_pts);
                Point pt = res.pt;
                double val = res.val;
                return py::make_tuple(pt,val);
            },
            py::arg("result_id"),
            py::arg("internal_pts")=10,
            R"pbdoc(
            Get maximum of a given data in domain.

            Parameters
            ------------
            result_id: ResultData
                ID of data.
            internal_pts: int, optional (default=10)
                Number of internal points per element.

            Returns
            -----------
            maxdata: tuple
                Tuple with point coordinates and value.
            )pbdoc")
        
        .def("get_min", 
            [](const StaticResult& self, ResultData id, int internal_pts) -> py::tuple {
                MaxMinResult res = self.get_min(id,internal_pts);
                Point pt = res.pt;
                double val = res.val;
                return py::make_tuple(pt,val);
            },
            py::arg("result_id"),
            py::arg("internal_pts")=10,
            R"pbdoc(
            Get minimum of a given data in domain.

            Parameters
            ------------
            result_id: ResultData
                ID of data.
            internal_pts: int, optional (default=10)
                Number of internal points per element.

            Returns
            -----------
            maxdata: tuple
                Tuple with point coordinates and value.
            )pbdoc")

        .def("element_mean", 
            &StaticResult::element_mean,
            py::arg("result_id"),
            py::arg("gauss_pts")=2,
            R"pbdoc(
            Get average of a given data in an element.

            Parameters
            ------------
            result_id: ResultData
                ID of data.
            gauss_pts: int, optional (default=2)
                Number of gauss points per element.

            Returns
            -----------
            mean: numpy.ndarray
                Vector with means per element.
            )pbdoc")
            

        .def("get_value", 
            &StaticResult::get_value,
            py::arg("result_id"),
            py::arg("loc"),
            R"pbdoc(
            Get the value of a parameter at a given point.

            Parameters
            ------------
            result_id: ResultData
                ID of data.
            loc: tuple (size 2 or 3)
                Coordinate of point.

            Returns
            -----------
            value: float
                Value of desired parameter at point.
            )pbdoc")

        .def("compliance", 
            &StaticResult::get_compliance,
            R"pbdoc(
            Get the compliance of this structure.

            Returns
            -----------
            value: float
                Compliance.
            )pbdoc")

        .def("compliance_sensitivity",
            &StaticResult::compliance_derivative,
            R"pbdoc(
            Get the sensitivity of the compliance of this structure for each element design variable.
            Uses the interpolation scheme defined in analysis.Analysis.

            Returns
            -----------
            alpha: numpy.ndarray
                Sensitivity of compliance.
            )pbdoc")
        ;
    }
    
    py::class_<StaticSolver, std::shared_ptr<StaticSolver>>(
        handle, 
        "StaticSolver",
        R"pbdoc(
        Static solver object.
        )pbdoc")

        .def(py::init<Analysis_ptr>(),
            py::arg("analysis"),
            R"pbdoc(
            Create a Static solver.

            Parameters
            ------------
            analysis: analysis.Analysis
                Analysis object to solve.
            )pbdoc")

        .def(py::init<Analysis_ptr, SolverType>(),
            py::arg("analysis"),
            py::arg("solver_type"),
            R"pbdoc(
            Create a Static solver.

            Parameters
            ------------
            analysis: analysis.Analysis
                Analysis object to solve.
            solver_type: SolverType
                Type of solver to use.
            )pbdoc")

        .def("solve",
            &StaticSolver::solve,
            R"pbdoc(
            Runs the solver.

            Returns
            ------------
            result: StaticResult
                StaticResult object that holds results.
            )pbdoc");
}