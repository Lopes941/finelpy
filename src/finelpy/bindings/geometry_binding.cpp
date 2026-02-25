#include <finelc/binding/bindings.h>
#include <finelc/binding/geometry_binding.h>
#include <finelc/geometry/geometry.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <iostream>
#include <vector>
#include <memory>


using namespace finelc;
namespace py = pybind11;




void bind_geometry(py::module_& handle){

    py::enum_<GeometryType>(handle, "GeometryType")

        .value("POLYGON", GeometryType::POLYGON)
        .value("RECTANGLE", GeometryType::RECTANGLE);

    {
    py::class_<IGeometry, std::shared_ptr<IGeometry>>
    (handle, "IGeometry")

        .def_property_readonly("nodes",
            [](IGeometry &self) -> py::array {

                int nnodes = (int)self.number_of_nodes();

                if(nnodes==0){
                    return py::none();
                }

                py::array_t<int> arr(nnodes);
                if(nnodes > 0){
                    std::memcpy(arr.mutable_data(), self.nodes().data(), sizeof(int) * nnodes);
                }
                return arr;

            })

        .def("is_inside", &IGeometry::is_inside)

        ;
    }

    {
    py::class_<Line, IGeometry, std::shared_ptr<Line>>
    (handle, "Line")

            .def(py::init<Point, Point>(),
                py::arg("point_1"),
                py::arg("point_2"))

            .def(py::init([](const double L) {
                Point p1(0,0,0);
                Point p2(L,0,0);
                return std::make_shared<Line>(p1,p2);
                }))

            .def(py::init([](const double x1, const double x2) {
                Point p1(x1,0,0);
                Point p2(x2,0,0);
                return std::make_shared<Line>(p1,p2);
                }))
    
            .def_property_readonly("len", &Line::length)
            .def_property_readonly("p1", [](const Line& self){return self.p1;})
            .def_property_readonly("p2", [](const Line& self){return self.p2;})
    ;
    }

        
    {
    py::class_<IArea, IGeometry, std::shared_ptr<IArea>>
    (handle, "IArea")

        .def(py::init<const std::vector<Point>&>())

        .def("name", 
            [](IArea& self){
                    return self.name();
            })

        .def_property_readonly("num_vertices",
            [](IArea &self) -> size_t {
                return self.get_num_vertices();
            })

        .def_property_readonly("vertices",
            [](IArea &self) -> std::vector<Point> {
                return *self.get_vertices();
            })

        .def_property_readonly("lines",
            [](IArea &self) -> std::vector<Line> {
                return *self.get_lines();
            })

        ;
    }

    {py::class_<Rectangle,IArea,std::shared_ptr<Rectangle>>
        (handle,"Rectangle")

        .def(py::init<Point, 
            Point>(),
            py::arg("dimensions"),
            py::arg("origin")=py::make_tuple(0,0),
            R"pbdoc(
            Create a rectangle with given dimensions and origin.

            Parameters
            ----------
            dimensions : list of float, length 2 or 3
                Dimensions lx, ly and t (thickness).
            origin: list of float, length 2 or 3, optional
                Optional origin for first point.
            )pbdoc"
        )
        
        .def_property_readonly("dimensions", &Rectangle::get_dimension)

        .def_property_readonly("origin", &Rectangle::get_origin)

        .def_static("static_name", &Rectangle::static_name)

        .def_property_readonly("lower_side", &Rectangle::lower_side)
        .def_property_readonly("right_side", &Rectangle::right_side)
        .def_property_readonly("upper_side", &Rectangle::upper_side)
        .def_property_readonly("left_side", &Rectangle::left_side)

        ;
    }

    {
    py::class_<IVolume, IGeometry, std::shared_ptr<IVolume>>
    (handle, "IVolume")

        .def("name", 
            [](IVolume& self){
                    return self.name();
            })

        .def_property_readonly("num_vertices",
            [](IVolume &self) -> size_t {
                return self.get_num_vertices();
            })

        .def_property_readonly("vertices",
            [](IVolume &self) -> std::vector<Point> {
                return *self.get_vertices();
            })

        .def_property_readonly("lines",
            [](IVolume &self) -> std::vector<Line> {
                return *self.get_lines();
            })

        .def_property_readonly("faces",
            [](IVolume &self) -> std::vector<IArea> {
                return *self.get_faces();
            })

        ;
    }

    {
    py::class_<Hexahedron, IVolume, std::shared_ptr<Hexahedron>>
    (handle, "Hexahedron")

        .def(py::init<Point, 
            Point>(),
            py::arg("dimensions"),
            py::arg("origin")=py::make_tuple(0,0),
            R"pbdoc(
            Create a hexahedron with given dimensions and origin.

            Parameters
            ----------
            dimensions : list of float, length 3
                Dimensions lx, ly and lz.
            origin: list of float, length 3, optional
                Optional origin for first point.
            )pbdoc"
        )
        
        .def_property_readonly("dimensions", &Hexahedron::get_dimension)

        .def_property_readonly("origin", &Hexahedron::get_origin)

        .def_static("static_name", &Hexahedron::static_name)

        .def_property_readonly("lower_face", &Hexahedron::lower_face)
        .def_property_readonly("front_face", &Hexahedron::front_face)
        .def_property_readonly("right_face", &Hexahedron::right_face)
        .def_property_readonly("back_face", &Hexahedron::back_face)
        .def_property_readonly("left_face", &Hexahedron::left_face)
        .def_property_readonly("upper_face", &Hexahedron::upper_face)
        ;
    }


}