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

    py::enum_<GeometryType>
        (handle, 
        "GeometryType",
        R"pbdoc(
        Represents a geometry type to be used by a mesher.
        )pbdoc")

        .value("POLYGON", GeometryType::POLYGON, R"pbdoc(Represents general 2D polygons.)pbdoc")
        .value("RECTANGLE", GeometryType::RECTANGLE, R"pbdoc(Represents a rectangle.)pbdoc");

    {
    py::class_<IGeometry, std::shared_ptr<IGeometry>>
        (handle, 
        "IGeometry",
        R"pbdoc(
        Interface for a geometry.
        )pbdoc")

        .def_property_readonly(
            "nodes",
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

            },
            R"pbdoc(
            Get the nodes associated with this geometry. Must be part of a mesh, otherwise it returns no nodes.

            Returns
            ----------
            nodes: numpy.ndarray
                Nodes from this geometry.
            )pbdoc"
        )

        .def_property_readonly(
            "number_of_nodes",
            &IGeometry::number_of_nodes,
            R"pbdoc(
            Get the number nodes associated with this geometry. Must be part of a mesh, otherwise it returns zero.

            Returns
            ----------
            num_nodes: int
                Number of nodes.
            )pbdoc"
        )

        .def("is_inside", 
            &IGeometry::is_inside,
            R"pbdoc(
            Check if a point is inside given geometry.

            Parameters
            ----------
            point: tuple or list, length 2 or 3
                Point to check if inside this geometry.

            Returns
            ----------
            inside: bool
                Boolean indicating whether point is inside.
            )pbdoc")

        ;
    }

    {
    py::class_<Line, IGeometry, std::shared_ptr<Line>>
        (handle, 
        "Line",
        R"pbdoc(
        Line object. Defined by two points.
        )pbdoc")

            .def(py::init<Point, Point>(),
                py::arg("p1"),
                py::arg("p2"),
                R"pbdoc(
                Create a Line object.

                Parameters
                ------------
                p1: tuple or list, lenght 2 or 3
                    First point.
                p2: tuple or list, lenght 2 or 3
                    Second point.

                )pbdoc")

            .def(py::init([](const double L) {
                Point p1(0,0,0);
                Point p2(L,0,0);
                return std::make_shared<Line>(p1,p2);
                }),
                py::arg("L"),
                R"pbdoc(
                Create a Line object from length. First point is (0,0,0).

                Parameters
                ------------
                L: float
                    Length of line.

                )pbdoc")

            .def(py::init([](const double x1, const double x2) {
                Point p1(x1,0,0);
                Point p2(x2,0,0);
                return std::make_shared<Line>(p1,p2);
                }),
                py::arg("p1"),
                py::arg("p2"),
                R"pbdoc(
                Create a Line on x axis with two points.

                Parameters
                ------------
                p1: float
                    First point x coordinate.
                p2: float
                    Second point x coordinate.

                )pbdoc")
    
            .def_property_readonly(
                "len",
                &Line::length,
                R"pbdoc(
                Get length of line.

                Returns
                ---------
                len: float
                    Length of line.
                )pbdoc")

            .def_property_readonly(
                "p1", 
                [](const Line& self){return self.p1;},
                R"pbdoc(
                Get point 1.

                Returns
                ---------
                p1: tuple
                    Point 1 from line.
                )pbdoc")

            .def_property_readonly(
                "p2", 
                [](const Line& self){return self.p2;},
                R"pbdoc(
                Get point 2.

                Returns
                ---------
                p1: tuple
                    Point 2 from line.
                )pbdoc")
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

    {
    py::class_<Rectangle,IArea,std::shared_ptr<Rectangle>>
        (handle,
        "Rectangle",
        R"pbdoc(
        Rectangle object. Defined by an origin point and dimensions.
        )pbdoc")

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
        
        .def_property_readonly(
            "dimensions", 
            &Rectangle::get_dimension,
            R"pbdoc(
            Get dimensions.

            Returns
            ---------
            dimensions: tuple
                Tuple with (lx, ly, t).
            )pbdoc"
            )

        .def_property_readonly(
            "origin", 
            &Rectangle::get_origin,
            R"pbdoc(
            Get origin point.

            Returns
            ---------
            origin: tuple
                Tuple origin point.
            )pbdoc")

        .def_static("static_name", &Rectangle::static_name)

        .def_property_readonly(
            "lower_side", 
            &Rectangle::lower_side,
            R"pbdoc(
            Get nodes on lower side of rectangle.

            Returns
            ---------
            nodes: numpy.ndarray
                Nodes on lower side.
            )pbdoc")

        .def_property_readonly(
            "right_side", 
            &Rectangle::right_side,
            R"pbdoc(
            Get nodes on right side of rectangle.

            Returns
            ---------
            nodes: numpy.ndarray
                Nodes on right side.
            )pbdoc")

        .def_property_readonly(
            "upper_side", 
            &Rectangle::upper_side,
            R"pbdoc(
            Get nodes on upper side of rectangle.

            Returns
            ---------
            nodes: numpy.ndarray
                Nodes on upper side.
            )pbdoc")

        .def_property_readonly(
            "left_side", 
            &Rectangle::left_side,
            R"pbdoc(
            Get nodes on left side of rectangle.

            Returns
            ---------
            nodes: numpy.ndarray
                Nodes on left side.
            )pbdoc"
        )

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
        (handle, 
        "Hexahedron",
        R"pbdoc(
        Hexahedron object. Defined by an origin point and dimensions.
        )pbdoc"
        )

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
        
        .def_property_readonly(
            "dimensions", 
            &Hexahedron::get_dimension,
            R"pbdoc(
            Get dimensions.

            Returns
            ---------
            dimensions: tuple
                Tuple with (lx, ly, lz).
            )pbdoc")

        .def_property_readonly(
            "origin", 
            &Hexahedron::get_origin,
            R"pbdoc(
            Get origin point.

            Returns
            ---------
            origin: tuple
                Tuple origin point.
            )pbdoc")

        .def_static("static_name", &Hexahedron::static_name)

        .def_property_readonly(
            "lower_face", 
            &Hexahedron::lower_face,
            R"pbdoc(
            Get nodes on lower face of hexahedron.

            Returns
            ---------
            nodes: numpy.ndarray
                Nodes on lower face.
            )pbdoc")

        .def_property_readonly(
            "front_face", 
            &Hexahedron::front_face,
            R"pbdoc(
            Get nodes on front face of hexahedron.

            Returns
            ---------
            nodes: numpy.ndarray
                Nodes on front face.
            )pbdoc")

        .def_property_readonly(
            "right_face", 
            &Hexahedron::right_face,
            R"pbdoc(
            Get nodes on right face of hexahedron.

            Returns
            ---------
            nodes: numpy.ndarray
                Nodes on right face.
            )pbdoc")

        .def_property_readonly(
            "back_face", 
            &Hexahedron::back_face,
            R"pbdoc(
            Get nodes on back face of hexahedron.

            Returns
            ---------
            nodes: numpy.ndarray
                Nodes on back face.
            )pbdoc")

        .def_property_readonly(
            "left_face", 
            &Hexahedron::left_face,
            R"pbdoc(
            Get nodes on left face of hexahedron.

            Returns
            ---------
            nodes: numpy.ndarray
                Nodes on left face.
            )pbdoc")

        .def_property_readonly(
            "upper_face", 
            &Hexahedron::upper_face,
            R"pbdoc(
            Get nodes on upper face of hexahedron.

            Returns
            ---------
            nodes: numpy.ndarray
                Nodes on upper face.
            )pbdoc")

        ;
    }


}