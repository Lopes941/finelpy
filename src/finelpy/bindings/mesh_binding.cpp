

#include <finelc/enumerations.h>
#include <finelc/mesh/mesh.h>
#include <finelc/mesh/meshers.h>

#include <finelc/elements/element.h>

#include <finelc/binding/bindings.h>
#include <finelc/binding/matrix_binding.h>
#include <finelc/binding/geometry_binding.h>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h> 
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

// #include <stdexcept>
#include <vector>
#include <memory>
#include <optional>
#include <cmath>

using namespace finelc;
namespace py = pybind11;




void bind_mesh(py::module_& handle){


    {
    py::class_<Mesh, Mesh_ptr>
        (handle, 
        "Mesh", 
        py::dynamic_attr(),
        R"pbdoc(
        Interface for mesh.
        )pbdoc")


        .def_property_readonly(
            "number_of_elements",
            &Mesh::number_of_elements,
            R"pbdoc(
            Number of elements in mesh.
            )pbdoc")

        .def_property_readonly(
            "number_of_nodes",
            &Mesh::number_of_nodes,
            R"pbdoc(
            Number of nodes in mesh.
            )pbdoc")

        .def_property_readonly(
            "nodes", 
            &Mesh::get_nodes,
            R"pbdoc(
            Array of nodes in mesh.
            )pbdoc")

        .def_property_readonly("elements", 
            [](const Mesh &self) {

            const VectorElements& elements = self.get_elements(); 

            // Create a numpy array of shape (N, 3)
            int n = elements.size();
            py::list els(n); 

            for (int i = 0; i < n; ++i) {
                els[i] = py::cast( elements[i], py::return_value_policy::reference_internal);
            }

            return els;
            },
            R"pbdoc(
            List of elements in mesh.
            )pbdoc")

        .def_property_readonly("element_nodes", 
            [](const Mesh &self) {

            VectorElements elements = self.get_elements(); 

            // Create a numpy array of shape (N, 3)
            int n = elements.size();
            py::list els(n); 

            for (int i = 0; i < n; ++i) {
                els[i] = py::cast(elements[i]->get_node_numbering(), py::return_value_policy::reference_internal);
            }

            return els;
            },
            R"pbdoc(
            List containing the node numbering of each element.
            )pbdoc")
    

        .def_property_readonly(
            "centers", 
            &Mesh::element_center,
            R"pbdoc(
            Array with coordinates of centroid for each element.
            )pbdoc")

        .def_property_readonly("ravel_elements",
            [](const Mesh &self) -> std::vector<std::vector<std::vector<double>>> {

                std::vector<std::vector<std::vector<double>>> element_ravel;
                element_ravel.reserve(self.number_of_elements());

                for(auto& el: self.get_elements()){

                    const VectorNodes& nodes = el->get_nodes();

                    std::vector<std::vector<double>> el_poly;
                    el_poly.reserve(nodes.size());

                    for(int n_vert=0; n_vert < nodes.size(); n_vert++){
                        el_poly.push_back({nodes[n_vert]->x, nodes[n_vert]->y});
                    }
                    element_ravel.emplace_back(std::move(el_poly));
                }

                return element_ravel;
            },
            R"pbdoc(
            3D array containing the nodes of each element. Each row contains a matrix, with the coordinates of the nodes of each element.
            Each row can be used by matplotlib to plot the geometry of each element.
            )pbdoc")

        .def_property_readonly("ravel_surfaces",
            [](const Mesh& self) -> py::tuple {

                std::vector<int> num_surfaces;
                num_surfaces.reserve(self.number_of_elements());

                int total_num = 0;
                for(auto el: self.get_elements()){
                    int nsurf = el->number_of_surfaces();
                    total_num += nsurf;
                    num_surfaces.emplace_back(nsurf);
                }

                std::vector<std::vector<std::vector<double>>> surface_ravel;
                surface_ravel.reserve(total_num);

                for(auto el: self.get_elements()){
                    for(auto surf : el->surfaces()){

                        const std::shared_ptr<std::vector<Point>>& vertices = surf->get_vertices();
                        std::vector<std::vector<double>> poly;
                        poly.reserve(vertices->size());

                        for(auto& vert : *vertices){
                            poly.push_back({vert.x,vert.y,vert.z});
                        }
                        surface_ravel.emplace_back(std::move(poly));
                    }
                }

                return py::make_tuple(num_surfaces,surface_ravel);
            },
            R"pbdoc(
            Tuple with an array of int and a 3D array. Each int represents the number of surfaces of the element. The 3D array gives the surfaces for each element. To be used in a 3D plot function.
            )pbdoc")
        
        .def("find_element", 
            &Mesh::find_element,
            py::arg("loc"),
            R"pbdoc(
            Find the element that contains the point.

            Parameters
            -----------
            loc: tuple (2 or 3 elements)
                Coordinate of point whose element is desired.

            Returns
            --------
            ind_el: int
                Index of found element. Returns -1 if no element contains point.
            )pbdoc"
            )

        .def("find_node", 
            &Mesh::find_node,
            R"pbdoc(
            Find the node the closest to given point.

            Parameters
            -----------
            loc: tuple (2 or 3 elements)
                Coordinate of point.

            Returns
            --------
            ind_el: int
                Index of found node.
            )pbdoc")

        ;
    }

    {
    py::class_<MeshBuilder, std::shared_ptr<MeshBuilder>>
        (handle, 
        "MeshBuilder",
        R"pbdoc(
        Interface for mesh builders.
        )pbdoc")

        .def("build", 
            &MeshBuilder::build,
            R"pbdoc(
            Generate the Mesh object.

            Returns
            --------
            mesh: Mesh
                Mesh object created by builder.

            )pbdoc")
        ;
    }

    {
    py::class_<MeshBuilderLine, MeshBuilder, std::shared_ptr<MeshBuilderLine>>
        (handle, 
        "LineMesh",
        R"pbdoc(
        Mesh generator for line mesh. A line mesh contains 1D elements disposed sequentialy.
        )pbdoc")

        .def(py::init<IGeometry_ptr, IElement_ptr>(),
             py::arg("line"),
             py::arg("element"),
            R"pbdoc(
            Start the creation of a Mesh using LineMesh logic.

            Parameters
            ------------
            line: geometry.Line
                Line object to build the line on.
            element: element.Element
                Element to use as base element for mesh generation.

            )pbdoc")

        .def("create_from_element_num", 
            &MeshBuilderLine::create_from_element_num,
            py::arg("num_el"),
            R"pbdoc(
            Set up mesh from number of elements. Elements are of same size.

            Parameters
            ------------
            num_el: int
                Number of element

            Returns
            ------------
            self: The current instance (for method chaining).

            )pbdoc")

            
        .def("create_from_element_size",
            &MeshBuilderLine::create_from_element_size,
            py::arg("dL"),
            R"pbdoc(
            Set up mesh from element size. Elements are of same size. If size does not fit the mesh, it is approximated to nearest fit.

            Parameters
            ------------
            dL: float
                Element size.

            Returns
            ------------
            self: The current instance (for method chaining).

            )pbdoc")

        ;
    }

    {
    py::class_<MeshBuilderFrame, MeshBuilder, std::shared_ptr<MeshBuilderFrame>>
        (handle, 
        "FrameMesh",
        R"pbdoc(
        Mesh generator for frame mesh. A frame mesh contains 1D elements disposed in 2D or 3D space, forming a truss or frame.
        )pbdoc")

        .def(py::init<IElement_ptr, VectorNodes, std::vector<std::vector<int>>>(),
            py::arg("element"),
            py::arg("node_coord"),
            py::arg("inci"),
            R"pbdoc(
            Start the creation of a Mesh using HexMesh logic.

            Parameters
            ------------
            element: element.Element
                Element to use as base element for mesh generation.
            node_coord: numpy.array
                Array with coordinates from each node.
            inci: numpy.array
                Array with nodal incidence on each element. Each row contains the node index from each element.

            )pbdoc")

        ;
    }

    {
    py::class_<MeshBuilderRectangle, MeshBuilder, std::shared_ptr<MeshBuilderRectangle>>
        (handle, 
        "RectangularMesh",
        R"pbdoc(
        Mesh generator for rectangular mesh. A rectangular mesh contains mapped 2D quadrilaterals or triangles.
        )pbdoc")

        .def(py::init<IGeometry_ptr, IElement_ptr>(),
            py::arg("domain"),
            py::arg("element"),
            R"pbdoc(
            Start the creation of a Mesh using RectangularMesh logic.

            Parameters
            ------------
            domain: geometry.IGeometry
                Geometry object to build the mesh on. Must be of Rectangle type.
            element: element.Element
                Element to use as base element for mesh generation.

            )pbdoc"
            )

        .def("set_element_size", 
            &MeshBuilderRectangle::set_element_size,
            R"pbdoc(
            Set up mesh from element dimensions. Elements are of same size. If size does not fit the mesh, it is approximated to nearest fit.

            Parameters
            ------------
            lx: float
                Element size on x axis.
            ly: float
                Element size on y axis.

            Returns
            ------------
            self: The current instance (for method chaining).

            )pbdoc")
            
        .def("set_grid",
            &MeshBuilderRectangle::set_grid,
            R"pbdoc(
            Set up mesh from element grid distribution. Elements are of same size.

            Parameters
            ------------
            nx: int
                Number of elements on x axis.
            ny: int
                Number of elements on y axis.

            Returns
            ------------
            self: The current instance (for method chaining).

            )pbdoc")
        ;
    }

    {
    py::class_<MeshBuilderHexahedron, MeshBuilder, std::shared_ptr<MeshBuilderHexahedron>>
        (handle, 
        "HexMesh",
        R"pbdoc(
        Mesh generator for hexahedral mesh. A hexahedral mesh contains mapped hexahedral elements.
        )pbdoc")

        .def(py::init<IGeometry_ptr, IElement_ptr>(),
            py::arg("domain"),
            py::arg("element"),
            R"pbdoc(
            Start the creation of a Mesh using HexMesh logic.

            Parameters
            ------------
            domain: geometry.IGeometry
                Geometry object to build the mesh on. Must be of Hexahedron type.
            element: element.Element
                Element to use as base element for mesh generation.

            )pbdoc")

        .def("set_element_size", 
            &MeshBuilderHexahedron::set_element_size,
            py::arg("lx"),
            py::arg("ly"),
            py::arg("lz"),
            R"pbdoc(
            Set up mesh from element dimensions. Elements are of same size. If size does not fit the mesh, it is approximated to nearest fit.

            Parameters
            ------------
            lx: float
                Element size on x axis.
            ly: float
                Element size on y axis.
            lz: float
                Element size on z axis.

            Returns
            ------------
            self: The current instance (for method chaining).

            )pbdoc")

            
        .def("set_grid",
            &MeshBuilderHexahedron::set_grid,
            R"pbdoc(
            Set up mesh from element grid distribution. Elements are of same size.

            Parameters
            ------------
            nx: int
                Number of elements on x axis.
            ny: int
                Number of elements on y axis.
            nz: int
                Number of elements on z axis.

            Returns
            ------------
            self: The current instance (for method chaining).

            )pbdoc")
        ;
    }

    

    {py::class_<NodeIterator,NodeIterator_ptr>
        (handle,"NodeIterator");
    }

}