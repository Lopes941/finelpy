#include <finelc/matrix.h>
#include <finelc/enumerations.h>

#include <finelc/elements/element.h>
#include <finelc/elements/element_matrix.h>
#include <finelc/elements/integration.h>
#include <finelc/elements/shape_func.h>

#include <finelc/binding/bindings.h>
#include <finelc/binding/matrix_binding.h>
#include <finelc/binding/geometry_binding.h>
#include <finelc/binding/element_binding.h>


#include <pybind11/pybind11.h>
#include <pybind11/numpy.h> 
#include <pybind11/stl.h>

#include <stdexcept>
#include <vector>
#include <memory>

using namespace finelc;
namespace py = pybind11;


void bind_element(py::module_& handle){

    {
    py::enum_<ShapeType>(
        handle, 
        "ShapeType",
        R"pbdoc(
        Represents the shape of the element.
        )pbdoc")

        .value("PYTHON_SHAPE", ShapeType::PYTHON_SHAPE, R"pbdoc(Python custom shape.)pbdoc")
        .value("LINE", ShapeType::LINE, R"pbdoc(Line element shape.)pbdoc")
        .value("TRI3", ShapeType::TRI3, R"pbdoc(Triangular with 3 nodes.)pbdoc")
        .value("QUAD4", ShapeType::QUAD4, R"pbdoc(Quadrangular with 4 nodes.)pbdoc")
        .value("QUAD9", ShapeType::QUAD9, R"pbdoc(Quadrangular with 9 nodes.)pbdoc")
        .value("HEX8", ShapeType::HEX8, R"pbdoc(Hexahedral with 8 nodes.)pbdoc")
        ;
    }

    {
    py::enum_<ModelType>(
        handle, 
        "ModelType",
        R"pbdoc(
        Represents the Physical model of the element.
        )pbdoc")

        .value("PYTHON_PHYSICS", ModelType::PYTHON_PHYSICS, R"pbdoc(Python custom physics.)pbdoc")
        .value("BAR_STRUCTURAL", ModelType::BAR_STRUCTURAL, R"pbdoc(Bar element physics.)pbdoc")
        .value("BEAM_STRUCTURAL", ModelType::BEAM_STRUCTURAL, R"pbdoc(Beam element physics.)pbdoc")
        .value("PLANE_STRUCTURAL", ModelType::PLANE_STRUCTURAL, R"pbdoc(Plane structural physics.)pbdoc")
        .value("SOLID_STRUCTURAL", ModelType::SOLID_STRUCTURAL, R"pbdoc(Solid 3D structural physics.)pbdoc")
        .value("THERMAL_CONDUCTION_1D", ModelType::THERMAL_CONDUCTION_1D, R"pbdoc(Plane wall thermal conduction model.)pbdoc")
        .value("THERMAL_CONDUCTION_2D", ModelType::THERMAL_CONDUCTION_2D, R"pbdoc(2D thermal conduction model.)pbdoc")
        .value("THERMAL_CONDUCTION_3D", ModelType::THERMAL_CONDUCTION_3D, R"pbdoc(3D thermal conduction model.)pbdoc")
        ;
    }

    {
    py::enum_<IntegrationGeometry>(
        handle, 
        "IntegrationGeometry",
        R"pbdoc(
        Identifier of geometry to choose quadrature.
        )pbdoc")

        .value("REGULAR", IntegrationGeometry::REGULAR, R"pbdoc(Regular domain: line, quadrangular, hexahedral.)pbdoc")
        .value("TRIANGLE", IntegrationGeometry::TRIANGLE, R"pbdoc(Triangle-like domain: triangular, tetrahedral.)pbdoc")
        ;
    }
    
    {
        py::class_<IElementShape, ElementShapeTrampoline, std::shared_ptr<IElementShape>>
        (handle, "ElementShape")

        .def(py::init<>())
        .def_property_readonly("shape", &IElementShape::shape)
        .def_property_readonly("integration_domain", &IElementShape::integration_domain)
        .def_property_readonly("dim", &IElementShape::number_of_dimensions)
        .def_property_readonly("shape_order", &IElementShape::shape_order)
        .def_property_readonly("number_of_nodes", &IElementShape::number_of_nodes)

        .def("J", &IElementShape::J,
            py::arg("element_nodes"),
            py::arg("loc")=py::none())

        .def("detJ", &IElementShape::detJ,
            py::arg("element_nodes"),
            py::arg("loc")=py::none())

        .def("N", &IElementShape::N,
            py::arg("loc"))

        .def("dNdxi", &IElementShape::dNdxi,
            py::arg("loc"))

        .def("dNdx", &IElementShape::dNdx,
            py::arg("element_nodes"),
            py::arg("loc"))
        ;
    }

    py::class_<IElementPhysics, std::shared_ptr<IElementPhysics>>
        (handle, "IElementPhysics");

    // {
    // py::class_<ElementalMatrices,std::shared_ptr<ElementalMatrices>>
    // (handle, "ElementalMatrices")
    //     .def(py::init<>())
    //     .def("get_Ke",
    //     [](ElementalMatrices& self, ))
    //     ;
    // }


    {
    py::class_<IElement, ElementTrampoline, std::shared_ptr<IElement>>
        (handle, 
        "Element", 
        py::dynamic_attr(),
        R"pbdoc(
        Interface for element.
        )pbdoc")

        .def(py::init<>())

        .def_property_readonly("shape", 
            &IElement::get_shape,
            R"pbdoc(Identifier of element shape type.)pbdoc")

        .def_property_readonly("model", 
            &IElement::get_model,
            R"pbdoc(Identifier of physics model type.)pbdoc")

        .def_property_readonly("constitutive", 
            &IElement::get_constitutive_model,
            R"pbdoc(Identifier of element constitutive type.)pbdoc")

        .def_property_readonly("nodes", 
            &IElement::get_nodes,
             R"pbdoc(Array of nodes from this element.)pbdoc")


        .def_property_readonly("dofs",
            &IElement::dofs,
            R"pbdoc(Degrees of freedom on this element. (See analysis.DOFType))pbdoc")

        .def_property_readonly("number_of_integration", 
            &IElement::get_number_integration_points,
            R"pbdoc(Number of integration points to be used in this element.)pbdoc")

        .def_property_readonly("edges", 
            &IElement::edges,
            R"pbdoc(Edges of this element as a geometry.Line object.)pbdoc")

        .def_property_readonly(
            "surfaces", 
            &IElement::surfaces,
            R"pbdoc(Surfaces of this element as a geometry.IGeometry object.)pbdoc")

        .def_property_readonly("number_of_nodes", 
            &IElement::number_of_nodes,
            R"pbdoc(Number of nodes in this element.)pbdoc")

        .def_property_readonly("dofs_per_node", 
            &IElement::dofs_per_node,
            R"pbdoc(Number of degrees of freedom per node.)pbdoc")

        .def_property_readonly("is_linear", 
            &IElement::is_linear,
            R"pbdoc(True if element is linear.)pbdoc")

        .def_property_readonly("linear_physics", 
            &IElement::linear_physics,
            R"pbdoc(True if physics from element is linear.)pbdoc")

        .def_property_readonly("linear_material", 
            &IElement::linear_material,
            R"pbdoc(True if material model from element is linear)pbdoc")
  

        /**********************SET UP METHODS****************************/

        .def_property_readonly("node_numbering", 
            &IElement::get_node_numbering,
            R"pbdoc(Node numbering from element, based on mesh definition.)pbdoc")

        .def("set_node_numbering", 
            py::overload_cast<const std::vector<int>&, const VectorNodes&>(&IElement::set_node_numbering),
            py::arg("node_numbering"),
            py::arg("mesh_nodes"),
            R"pbdoc(
            Set up node numbering from this node and includes the mesh nodes.

            Parameters
            -----------
            node_numbering: array of ints
                Array with node numbering from this element, based on mesh definition.
            mesh_nodes: array of Points (tuple of 3 elements)
                Nodes from the mesh.
            )pbdoc")

        .def("set_node_numbering",
            py::overload_cast<const std::vector<int>&>(&IElement::set_node_numbering),
            py::arg("node_numbering"),
            R"pbdoc(
            Set up node numbering from this node.

            Parameters
            -----------
            node_numbering: array of ints
                Array with node numbering from this element, based on mesh definition.
            )pbdoc")

        .def("copy", 
            &IElement::copy,
            py::arg("same_matrix"),
            R"pbdoc(
            Copy the element.

            Parameters
            -----------
            same_matrix: bool, optional (defult=false)
                Flag if the new element will use the same matrices (only for linear element).

            Returns
            -----------
            element_copy: Element
                Element with exact copy.
            )pbdoc")

        .def("copy_to", 
            &IElement::copy_to,
            py::arg("other_element"),
            R"pbdoc(
            Copy the element to another given object.

            Parameters
            -----------
            other_element: Element
                Object which will receive the copy of the current element.
            )pbdoc")
        

        /**********************SHAPE METHODS****************************/

        .def("J", 
            &IElement::J,
            py::arg("loc")=py::none(),
            R"pbdoc(
            Get the Jacobian of element at given local coordinate.

            Parameters
            -----------
            loc: tuple (2 or 3 elements)
                Local coordinates where the value is computed.

            Returns
            --------
            J: numpy.array
                Jacobian matrix.
            )pbdoc")

        .def("detJ", 
            &IElement::detJ,
            py::arg("loc")=py::none(),
            R"pbdoc(
            Get the determinant of the Jacobian of element at given local coordinate.

            Parameters
            -----------
            loc: tuple (2 or 3 elements)
                Local coordinates where the value is computed.

            Returns
            --------
            detJ: float
                Determinant of Jacobian matrix.
            )pbdoc")

        .def("N_shape", 
            &IElement::N_shape,
            py::arg("loc"),
            R"pbdoc(
            Get the shape functions of the element evaluated at given local coordinate.
            Note the difference between the shape functions (geometric) and the interpolation
            functions (physics).

            Parameters
            -----------
            loc: tuple (2 or 3 elements)
                Local coordinates where the value is computed.

            Returns
            --------
            N: numpy.array
                Vector with evaluated shape functions.
            )pbdoc")

        // .def("local_to_global",
        //     py::overload_cast<const Point&>(&IElement::local_to_global),
        //     py::arg("loc"))

        // .def("local_to_global",
        //     py::overload_cast<const Vector&>(&IElement::local_to_global),
        //     py::arg("loc"),
        //     R"pbdoc(
        //     Map a point given in local variables to global variables.

        //     Parameters
        //     -----------
        //     loc: tuple (2 or 3 elements)
        //         Local coordinates where the value is computed.
        //     tol: float, optional (default=1e-7)
        //         Tolerance of final value.

        //     Returns
        //     --------
        //     gloc: tuple
        //         Location of input point in global variables.
        //     )pbdoc")

        

        .def("dNdxi_shape", 
            &IElement::dNdxi_shape,
            py::arg("loc"),
            R"pbdoc(
            Get the derivatives of shape functions of the element with respect to LOCAL VARIABLES
            evaluated at given local coordinate.
            Note the difference between the shape functions (geometric) and the interpolation
            functions (physics).

            Parameters
            -----------
            loc: tuple (2 or 3 elements)
                Local coordinates where the value is computed.

            Returns
            --------
            dN: numpy.array
                Vector with evaluated shape functions derivatives.
            )pbdoc")

        .def("dNdx_shape", 
            &IElement::dNdx_shape,
            py::arg("loc"),
            R"pbdoc(
            Get the derivatives of shape functions of the element with respect to GLOBAL VARIABLES
            evaluated at given local coordinate.
            Note the difference between the shape functions (geometric) and the interpolation
            functions (physics).

            Parameters
            -----------
            loc: tuple (2 or 3 elements)
                Local coordinates where the value is computed.

            Returns
            --------
            dN: numpy.array
                Vector with evaluated shape functions derivatives.
            )pbdoc")

        .def("global_to_local", 
            &IElement::global_to_local,
            py::arg("gloc"),
            py::arg("tol")=1e-7,
            R"pbdoc(
            Map a point given in global variables to local variables.

            Parameters
            -----------
            gloc: tuple (2 or 3 elements)
                Global coordinates where the value is computed.
            tol: float, optional (default=1e-7)
                Tolerance of final value.

            Returns
            --------
            gloc: tuple
                Location of input point in local variables.
            )pbdoc")


        /******************CONSTITUTIVE METHODS************************/

        .def("D", &IElement::D,
            py::arg("loc")=py::none(),
            py::arg("displacement")=py::none(),
            R"pbdoc(
            Get the constitutive matrix of the element at given point.

            Parameters
            -----------
            loc: tuple (2 or 3 elements)
                Local coordinates where the value is computed.
            displacement: array, optional (default=None)
                Nodal displacements of element, used in non-linear analyses.

            Returns
            --------
            D: numpy.array
                Constitutive matrix of element evaluated at given point.
            )pbdoc")
        

        /*********************PHYSICS METHODS**************************/

        .def("N", &IElement::N,
            py::arg("loc"),
            py::arg("displacement")=py::none(),
            R"pbdoc(
            Get the interpolation functions of the element.
            Note the difference between the shape functions (geometric) and the interpolation
            functions (physics).

            Parameters
            -----------
            loc: tuple (2 or 3 elements)
                Local coordinates where the value is computed.
            displacement: array, optional (default=None)
                Nodal displacements of element, used in non-linear analyses.

            Returns
            --------
            N: numpy.array
                Vector with evaluated interpolation functions.
            )pbdoc")

        .def("dNdx", &IElement::dNdx,
            py::arg("loc"),
            py::arg("displacement")=py::none(),
            R"pbdoc(
            Get the derivatives of interpolation functions of the element with respect to GLOBAL VARIABLES
            evaluated at given local coordinate.
            Note the difference between the shape functions (geometric) and the interpolation
            functions (physics).

            Parameters
            -----------
            loc: tuple (2 or 3 elements)
                Local coordinates where the value is computed.
            displacement: array, optional (default=None)
                Nodal displacements of element, used in non-linear analyses.

            Returns
            --------
            dN: numpy.array
                Vector with evaluated interpolation functions derivatives.
            )pbdoc")

        .def("B", &IElement::B,
            py::arg("loc"),
            py::arg("displacement")=py::none(),
            R"pbdoc(
            Get the strain-displacement matrix at a given location.

            Parameters
            -----------
            loc: tuple (2 or 3 elements)
                Local coordinates where the value is computed.
            displacement: array, optional (default=None)
                Nodal displacements of element, used in non-linear analyses.

            Returns
            --------
            B: numpy.array
                Strain-displacement matrix.
            )pbdoc")

        .def("Ke", 
            &IElement::Ke,
            py::arg("displacement")=py::none(),
            R"pbdoc(
            Get the stiffness matrix of element.

            Parameters
            -----------
            displacement: array, optional (default=None)
                Nodal displacements of element, used in non-linear analyses.

            Returns
            --------
            Ke: numpy.array
                Stiffness matrix of element.
            )pbdoc")

        .def("Me", 
            &IElement::Me,
            py::arg("displacement")=py::none(),
            R"pbdoc(
            Get the mass matrix of element.

            Parameters
            -----------
            displacement: array, optional (default=None)
                Nodal displacements of element, used in non-linear analyses.

            Returns
            --------
            Ke: numpy.array
                Mass matrix of element.
            )pbdoc")


        /************************INTEGRATION METHODS*****************************/

        .def("integration_pair", 
            [](const IElement& self, int num_points = 0) -> py::list {
                std::vector<PointWeight> pairs = self.integration_pair(num_points);

                py::list pair_list;

                for(auto& pair:pairs){
                    pair_list.append(py::make_tuple(pair.point, pair.weight));
                }

                return pair_list;

            },
            py::arg("num_points")=0,
            R"pbdoc(
            Get the integration points and weights in this element.

            Parameters
            ------------
            number_of_points: int, optional (default=0)
                Number of integration points to evaluate. Defaults to value provided by Physics.

            Returns
            --------
            pair: array of tuples
                Array with integration point pair. Each value contains the integration point coordinates and its weight.

            )pbdoc")

        .def_property_readonly("integration_points", 
            &IElement::integration_points,
            R"pbdoc(
            Get the integration points in this element.
            )pbdoc"
        )

        .def_property_readonly("integration_weights", 
            &IElement::integration_weights,
            R"pbdoc(
            Get the integration weights in this element.
            )pbdoc")

        .def("set_number_integration_points", 
            &IElement::set_number_integration_points,
            py::arg("num_points"),
            R"pbdoc(
            Set the number of integration points in this element.

            Parameters
            ------------
            number_of_points: int
                Number of integration points to evaluate.

            )pbdoc"
        )
        

        /*********************RESULT ACCESS METHODS**************************/

        // double compute_double(ResultData data_name, )

        // ELASTICITY
        .def_property_readonly("supports_strain", 
            &IElement::supports_strain,
            R"pbdoc(
            Get if element supports evaluation of strain.
            )pbdoc")

        .def("get_strain", 
            &IElement::get_strain,
            py::arg("loc"),
            py::arg("displacement"),
            R"pbdoc(
            Get the strain (all 6 components) at given point in local coordinates.

            Parameters
            -----------
            loc: tuple (2 or 3 elements)
                Local coordinates where the value is computed.
            displacement: array
                Nodal displacements of element, used in non-linear analyses.

            Returns
            --------
            val: numpy.array
                Strain computed at point.
            )pbdoc"
        )

        .def_property_readonly("supports_stress", 
            &IElement::supports_stress,
            R"pbdoc(
            Get if element supports evaluation of stress.
            )pbdoc")

        .def("get_stress", 
            &IElement::get_stress,
            py::arg("loc"),
            py::arg("displacement"),
            R"pbdoc(
            Get the stress (all 6 components) at given point in local coordinates.

            Parameters
            -----------
            loc: tuple (2 or 3 elements)
                Local coordinates where the value is computed.
            displacement: array
                Nodal displacements of element, used in non-linear analyses.

            Returns
            --------
            val: numpy.array
                Stress computed at point.
            )pbdoc"
        )
        ;

    }

    
    {
    handle.def("get_integration_points",
        [](int number_of_points, IntegrationGeometry geo = IntegrationGeometry::REGULAR,int dimensions=1) -> py::tuple {

            std::vector<PointWeight> pair = get_gauss_points(geo,number_of_points,dimensions);

            py::tuple out(pair.size());
            for(int i=0; i<pair.size(); i++){
                out[i] = py::make_tuple(pair[i].point,pair[i].weight);
            }

            return out;

        },
        py::arg("number_of_points"),
        py::arg("geometry") = IntegrationGeometry::REGULAR,
        py::arg("dimensions") = 1,
        R"pbdoc(
        Get the integration points and weights for a given number of points.

        Parameters
        ------------
        number_of_points: int
            Number of integration points to evaluate.
        geometry: IntegrationGeometry (optional, default=REGULAR)
            Geometry of the area to be integrated.
        dimensions: int (optional, default=1)
            Number of dimensions of geometry.

        Returns
        --------
        pair: array of tuples
            Array with integration point pair. Each value contains the integration point coordinates and its weight.

        )pbdoc");
    }

    {
    handle.def("eval_lagrange", &eval_lagrange_polynomials,
        py::arg("xi"),
        py::arg("order"),
        R"pbdoc(
        Evaluate the lagrange polynomials of a given order defined at the interval (-1,1) at a given point.
        The Lagrange polynomials are ordered so the ones at the points -1 and 1 are the first. The polynomials
        defined at the other points are given afterward.

        Parameters
        ------------
        xi: float (between -1 and 1)
            Coordinate to evaluate the polynomials.
        order: int
            Order of lagrange polynomials.

        Returns
        --------
        N: array
            Array with value of lagrange polynomials.

        )pbdoc");
    }

    {
    handle.def("eval_lagrange_derivative", &eval_lagrange_polynomials_derivatives,
        py::arg("xi"),
        py::arg("order"),
        R"pbdoc(
        Evaluate the derivative of lagrange polynomials of a given order defined at the interval (-1,1) at a given point.
        The Lagrange polynomials are ordered so the ones at the points -1 and 1 are the first. The polynomials
        defined at the other points are given afterward.

        Parameters
        ------------
        xi: float (between -1 and 1)
            Coordinate to evaluate the polynomials.
        order: int
            Order of lagrange polynomials.

        Returns
        --------
        dN: array
            Array with the derivatives of lagrange polynomials.

        )pbdoc");
    }

    {
    handle.def("create_element",
            &create_element,
            py::arg("shape_type"),
            py::arg("model_type"),
            py::arg("constitutive_type"),
            py::arg("material")=py::none(),
            py::return_value_policy::automatic_reference,
            R"pbdoc(
            Create an element with given shape, physics and constitutive models.

            Parameters
            ------------
            shape_type: ShapeType
                Identifier to shape type of element.
            model_type: ModelType
                Identifier to physics type of element.
            constitutive_type: ConstitutiveType
                Identifier to constitutive type of element.
            material: material.Material (optional, but most elements demand a material model.)
                Material that constitutes the element.

            Returns
            --------
            el: Element
                Created element. Returned by reference.

            )pbdoc"
        );
    }

    {
    handle.def("move_element_to_Cpp",
        [](const py::object& el) -> IElement_ptr {
            auto cpp = el.cast<IElement_ptr>();
            cpp->py_self = std::make_unique<py::object>(el);
            return cpp;
        },
        R"pbdoc(
        Moves an element definition to C++. Used on custom element definitions.

        Parameters
        ------------
        element: python object
            Custom python defined element.

        Returns
        --------
        el: Element
            Created element.

        )pbdoc");
    }

}