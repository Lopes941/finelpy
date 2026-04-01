#include <finelc/matrix.h>
#include <finelc/enumerations.h>

#include <finelc/analysis/analysis.h>
#include <finelc/analysis/interpolation.h>

#include <finelc/binding/bindings.h>
#include <finelc/binding/matrix_binding.h>
#include <finelc/binding/geometry_binding.h>


#include <pybind11/pybind11.h>
#include <pybind11/numpy.h> 
#include <pybind11/stl.h>

#include <stdexcept>
#include <vector>
#include <memory>

using namespace finelc;
namespace py = pybind11;

void bind_analysis(py::module_& handle){

    {
    py::enum_<DOFType>(
        handle, 
        "DOFType",
        R"pbdoc(
        Represents the degrees of freedom in nodes.
        )pbdoc")
        .value("UX", DOFType::UX, R"pbdoc(Displacement in X direction.)pbdoc")
        .value("UY", DOFType::UY, R"pbdoc(Displacement in Y direction.)pbdoc")
        .value("UZ", DOFType::UZ, R"pbdoc(Displacement in Z direction.)pbdoc")
        .value("THETAX", DOFType::THETAX, R"pbdoc(Rotation in X direction.)pbdoc")
        .value("THETAY", DOFType::THETAY, R"pbdoc(Rotation in Y direction.)pbdoc")
        .value("THETAZ", DOFType::THETAZ, R"pbdoc(Rotation in Z direction.)pbdoc")
        .value("T", DOFType::T, R"pbdoc(Temperature.)pbdoc")
        .value("P", DOFType::P, R"pbdoc(Pressure.)pbdoc")
        ;
    }

    {
    py::enum_<InterpolationScheme>(
        handle, 
        "InterpolationScheme",
        R"pbdoc(
        Represents the type of interpolation scheme used in the update procedure.
        )pbdoc")
        .value("NONE", InterpolationScheme::NONE, R"pbdoc(No interpolation.)pbdoc")
        .value("SIMP", InterpolationScheme::SIMP, R"pbdoc(Solid Isotropic with Material Penalization.)pbdoc")
        ;
    }

    {
    py::enum_<InterpolationParameters>(
        handle, 
        "InterpolationParameters",
        R"pbdoc(
        Represents the parameters from interpolation schemes.
        )pbdoc")
        .value("P_EXPONENT", InterpolationParameters::P_EXPONENT, R"pbdoc(Exponent in exponential laws.)pbdoc")
        .value("X_MIN", InterpolationParameters::X_MIN, R"pbdoc(Void element pseudo-density.)pbdoc")
        .value("BETA", InterpolationParameters::BETA, R"pbdoc(Sigmoid beta.)pbdoc")
        .value("THRESHOLD", InterpolationParameters::THRESHOLD, R"pbdoc(Threshold for sigmoid.)pbdoc")
        ;
    }

    py::class_<IInterpolationScheme, std::shared_ptr<IInterpolationScheme>>(handle,"IInterpolationScheme");
    
    {
    py::class_<Analysis, std::shared_ptr<Analysis>>(
        handle, 
        "Analysis",
        R"pbdoc(Interface for Analysis)pbdoc")

        .def_property_readonly("ID",
        [](const Analysis& self) -> Matrix {

            const IDMat& orig_ID = self.get_ID();

            Matrix ID(orig_ID.rows,orig_ID.cols);

            for(int i=0; i<orig_ID.rows; i++){
                for(int j=0; j<orig_ID.cols; j++){
                    ID(i,j) = orig_ID(i,j);
                }
            }
            return ID;
        },
        R"pbdoc(Identification matrix. 
        The ID matrix is a number_of_dofs x number_of_nodes matrix correlating each dof to the index of the global matrices and vectors.)pbdoc")

        .def_property_readonly("mesh", 
            &Analysis::get_mesh,
            R"pbdoc(Mesh object in this analysis.)pbdoc"
        )

        .def_property_readonly("num_bc_dofs", 
            &Analysis::bc_size,
            R"pbdoc(Number of dofs with boundary conditions)pbdoc"
        )

        .def_property_readonly("num_free_dofs", 
            &Analysis::get_size,
            R"pbdoc(Number of free degrees of freedom)pbdoc"
        )

        .def_property_readonly("num_total_dofs", 
            &Analysis::total_size,
            R"pbdoc(Total number of dofs.)pbdoc"
        )

        .def_property_readonly("interpolation_scheme", 
            &Analysis::get_interpolation_name,
            R"pbdoc(Interpolation Scheme adopted in the mesh update procedure.)pbdoc"
        )

        .def_property_readonly("rho", 
            &Analysis::get_pseudo_density,
            R"pbdoc(Pseudo-density of the current analysis.)pbdoc")
        
        .def_property_readonly("free_dofs", 
            [](const Analysis& self) -> py::array_t<int> {
                return py::cast(self.get_free_dofs());
            },
            R"pbdoc(Index of free degrees of freedom.)pbdoc")

        .def_property_readonly("bc_dofs", 
            [](const Analysis& self) -> py::array_t<int> {
                return py::cast(self.get_bc_dofs());
            },
            R"pbdoc(Index of degrees of freedom with boundary conditions.)pbdoc")

        .def("destroy", 
            &Analysis::destroy,
            R"pbdoc(Destructor of the Analysis object.)pbdoc")

        .def("set_interpolation", 
            &Analysis::set_interpolation,
            py::arg("name"),
            R"pbdoc(
            Set up the interpolation used in the mesh update procedure.

            Parameters
            -----------
            name: InterpolationScheme
                Identification of the interpolation scheme used.
            )pbdoc"
        )

        .def("update_interpolation",
            &Analysis::update_interpolation,
            py::arg("identifier"),
            py::arg("value"),
            R"pbdoc(
            Updates a parameter from the interpolation scheme.

            Parameters
            ------------
            identifier: InterpolationParameters
                Identifier of the parameter to be updated.
            value: float
                New value.
            )pbdoc")

        .def("update_pseudo_density", 
            py::overload_cast<double>(&Analysis::update_pseudo_density),
            py::arg("new_rho"),
            R"pbdoc(
            Update the pseudo density vector with a constant vector of given value.

            Parameters
            ------------
            new_rho: float
                New value.
            )pbdoc")

        .def("update_pseudo_density", 
            py::overload_cast<const Vector&>(&Analysis::update_pseudo_density),
            py::arg("new_rho"),
            R"pbdoc(
            Update the pseudo density vector with a constant vector of given value.

            Parameters
            ------------
            new_rho: numpy.array
                New value.
            )pbdoc")

        .def("Kg", 
            &Analysis::Kg,
            R"pbdoc(
            Get the global stiffness matrix.

            Returns
            -----------
            Kg: numpy.ndarray
                Global stiffness matrix.
            )pbdoc")

        .def("Mg", 
            &Analysis::Mg,
            R"pbdoc(
            Get the global mass matrix.

            Returns
            -----------
            Mg: numpy.ndarray
                Global mass matrix.
            )pbdoc")

        .def("fg", 
            &Analysis::fg,
            R"pbdoc(
            Get the global force vector.

            Returns
            -----------
            fg: numpy.ndarray
                Global force vector.
            )pbdoc")
        ;
    }

    py::class_<AnalysisBuilder, std::shared_ptr<AnalysisBuilder>>(
        handle, 
        "AnalysisBuilder",
        R"pbdoc(
        Builder class for Analysis.
        )pbdoc")

        .def(py::init<Mesh_ptr>(),
            py::arg("mesh"),
            R"pbdoc(
            Create AnalysisBuilder object.

            Parameters
            ------------
            mesh: mesh.Mesh
                Reference mesh.
            )pbdoc")

        .def("add_boundary_condition", 
            py::overload_cast<DOFType, int, double>(&AnalysisBuilder::add_boundary_condition),
            py::arg("dof"),
            py::arg("node_number"),
            py::arg("value"),
            R"pbdoc(
            Add one Dirichlet boundary condition to analysis.

            Parameters
            ------------
            dof: DOFType
                Degree of Freedom ID.
            node_number: int
                ID of node where it is applied.
            value: float
                Value of boundary condition.
            )pbdoc")

        .def("add_boundary_condition", 
            py::overload_cast<DOFType, std::vector<int>, double>(&AnalysisBuilder::add_boundary_condition),
            py::arg("dof"),
            py::arg("node_number"),
            py::arg("value"),
            R"pbdoc(
            Add same Dirichlet boundary condition to multiple dofs to analysis.

            Parameters
            ------------
            dof: DOFType
                Degree of Freedom ID.
            node_number: numpy.array[int]
                ID of nodes where it is applied.
            value: float
                Value of boundary condition.
            )pbdoc")

        .def("add_boundary_condition", 
            py::overload_cast<DOFType, IGeometry_ptr, double>(&AnalysisBuilder::add_boundary_condition),
            py::arg("dof"),
            py::arg("geometry"),
            py::arg("value"),
            R"pbdoc(
            Add Dirichlet boundary to a geometry.

            Parameters
            ------------
            dof: DOFType
                Degree of Freedom ID.
            geometry: geometry.IGeometry
                Location where boundary condition is applied.
            value: float
                Value of boundary condition.
            )pbdoc")

        .def("add_force", 
            py::overload_cast<DOFType, int, double>(&AnalysisBuilder::add_force),
            py::arg("dof"),
            py::arg("node_number"),
            py::arg("value"),
            R"pbdoc(
            Add Neumann boundary condition to a node.

            Parameters
            ------------
            dof: DOFType
                Degree of Freedom ID.
            node_number: int
                ID of node where it is applied.
            value: float
                Value of force.
            )pbdoc")

        .def("add_force", 
            py::overload_cast<DOFType, std::vector<int>, double>(&AnalysisBuilder::add_force),
            py::arg("dof"),
            py::arg("node_number"),
            py::arg("value"),
            R"pbdoc(
            Add same Neumann boundary condition to multiple dofs.

            Parameters
            ------------
            dof: DOFType
                Degree of Freedom ID.
            node_number: numpy.array[int]
                ID of nodes where it is applied.
            value: float
                Value of force.
            )pbdoc")

        .def("add_force", 
            py::overload_cast<DOFType, IGeometry_ptr, double, int>(&AnalysisBuilder::add_force),
            py::arg("dof"),
            py::arg("geometry"),
            py::arg("value"),
            py::arg("integration_points")=10,
            R"pbdoc(
            Add uniform Neumann boundary condition to a geometry.

            Parameters
            ------------
            dof: DOFType
                Degree of Freedom ID.
            geometry: geometry.IGeometry
                Location where boundary condition is applied.
            value: float
                Value of uniform force.
            integration_points: int, optional (default=10)
                Number of integration points to be used to integrate the force.
            )pbdoc")

        .def("add_force", 
            py::overload_cast<DOFType, IGeometry_ptr, Evalfn, int>(&AnalysisBuilder::add_force),
            py::arg("dof_type"),
            py::arg("domain"),
            py::arg("function"),
            py::arg("number_of_integration")=10,
            R"pbdoc(
            Add general Neumann boundary condition to a geometry.

            Parameters
            ------------
            dof: DOFType
                Degree of Freedom ID.
            geometry: geometry.IGeometry
                Location where boundary condition is applied.
            function: function handle ( tuple[2,3] -> float )
                Function that calculates the force with given coordinates.
            integration_points: int, optional (default=10)
                Number of integration points to be used to integrate the force.
            )pbdoc")

        .def("build", 
            &AnalysisBuilder::build,
            R"pbdoc(
            Generate the Analysis object.

            Returns
            --------
            analysis: Analysis
                Analysis object created by builder.

            )pbdoc")

        ;

}