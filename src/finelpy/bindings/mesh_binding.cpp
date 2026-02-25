

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
    (handle, "Mesh", py::dynamic_attr())


        .def_property_readonly("number_of_elements",
            &Mesh::number_of_elements)

        .def_property_readonly("number_of_nodes",
            &Mesh::number_of_nodes)

        .def_property_readonly("nodes", &Mesh::get_nodes)

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
        })

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
        })
    

        .def_property_readonly("centers", &Mesh::element_center)
        
        .def("find_element", &Mesh::find_element)

        .def("find_node", &Mesh::find_node)

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
            })

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
            })

        ;
    }

    {
    py::class_<MeshBuilder, std::shared_ptr<MeshBuilder>>
    (handle, "MeshBuilder")

        .def("build", 
            &MeshBuilder::build)
        ;
    }

    {
    py::class_<MeshBuilderLine, MeshBuilder, std::shared_ptr<MeshBuilderLine>>
    (handle, "LineMesh")

        .def(py::init<IGeometry_ptr, IElement_ptr>(),
             py::arg("line"),
             py::arg("element"))

        .def("create_from_element_num", 
            &MeshBuilderLine::create_from_element_num)

            
        .def("create_from_element_size",
            &MeshBuilderLine::create_from_element_size)

        ;
    }

    {
    py::class_<MeshBuilderRectangle, MeshBuilder, std::shared_ptr<MeshBuilderRectangle>>
    (handle, "RectangularMesh")

        .def(py::init<IGeometry_ptr, IElement_ptr>(),
             py::arg("domain"),
             py::arg("element"))

        .def("set_element_size", 
            &MeshBuilderRectangle::set_element_size)

            
        .def("set_grid",
            &MeshBuilderRectangle::set_grid)
        ;
    }

    {
    py::class_<MeshBuilderHexahedron, MeshBuilder, std::shared_ptr<MeshBuilderHexahedron>>
    (handle, "HexMesh")

        .def(py::init<IGeometry_ptr, IElement_ptr>(),
             py::arg("domain"),
             py::arg("element"))

        .def("set_element_size", 
            &MeshBuilderHexahedron::set_element_size)

            
        .def("set_grid",
            &MeshBuilderHexahedron::set_grid)
        ;
    }

    {
    py::class_<MeshBuilderFrame, MeshBuilder, std::shared_ptr<MeshBuilderFrame>>
    (handle, "FrameMesh")

        .def(py::init<IElement_ptr, VectorNodes, std::vector<std::vector<int>>>(),
            py::arg("element"),
            py::arg("node_coord"),
            py::arg("inci"))
        ;
    }

    {py::class_<NodeIterator,NodeIterator_ptr>
        (handle,"NodeIterator");
    }

}