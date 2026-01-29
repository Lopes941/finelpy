#pragma once

#include <finelc/geometry/geometry.h>

#include <finelc/binding/matrix_binding.h>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h> 

#include <vector>
#include <memory>


namespace py = pybind11;

namespace pybind11 { namespace detail {

     template <> struct type_caster<finelc::Point> {

        public:

            PYBIND11_TYPE_CASTER(finelc::Point, _("Tuple[float, float] or Tuple[float, float, float]"));

            // Python -> C++
            bool load(handle src, bool){

                if (py::isinstance<py::tuple>(src)){
                    auto tup = reinterpret_borrow<py::tuple>(src);

                    double x = tup[0].cast<double>();
                    double y = 0;
                    double z = 0;
                    if (tup.size() >= 3){
                        y = tup[1].cast<double>();
                        z = tup[2].cast<double>();
                    }else if(tup.size() >=2 ){
                        y = tup[1].cast<double>();
                    };
                    value = finelc::Point(x,y,z);
                    return true;
                } 
                
                if (py::isinstance<py::float_>(src) || py::isinstance<py::int_>(src)) {
                    double x = src.cast<double>();
                    value = finelc::Point(x,0,0);
                    return true;
                }
                
                 return false;
            }

            // C++ -> Python
            static handle cast(const finelc::Point& src, return_value_policy policy, handle){

                py::object py_x = py::cast(src.x,policy);
                py::object py_y = py::cast(src.y,policy);
                py::object py_z = py::cast(src.z,policy);

                return py::make_tuple(py_x, py_y, py_z).release();
            }

    };

    template <> struct type_caster<finelc::Node_ptr> {

        public:

            PYBIND11_TYPE_CASTER(finelc::Node_ptr, _("Tuple[float, float] or Tuple[float, float, float]"));

            // Python -> C++
            bool load(handle src, bool){

                if (py::isinstance<py::tuple>(src)){
                    auto tup = reinterpret_borrow<py::tuple>(src);

                    double x = tup[0].cast<double>();
                    double y = 0;
                    double z = 0;
                    if (tup.size() >= 3){
                        y = tup[1].cast<double>();
                        z = tup[2].cast<double>();
                    }else if(tup.size() >=2 ){
                        y = tup[1].cast<double>();
                    };
                    value = std::make_shared<finelc::Node>(x,y,z);
                    return true;
                } 
                
                if (py::isinstance<py::float_>(src) || py::isinstance<py::int_>(src)) {
                    double x = src.cast<double>();
                    value = std::make_shared<finelc::Node>(x,0,0);
                    return true;
                }
                
                 return false;
            }

            // C++ -> Python
            static handle cast(const finelc::Node_ptr& src, return_value_policy policy, handle){

                py::object py_x = py::cast(src->x,policy);
                py::object py_y = py::cast(src->y,policy);
                py::object py_z = py::cast(src->z,policy);

                return py::make_tuple(py_x, py_y, py_z).release();
            }

    };


    template <> struct type_caster<finelc::VectorNodes> {

        public:

            PYBIND11_TYPE_CASTER(finelc::VectorNodes, _("numpy.ndarray[float64, shape=(n,3)]"));

            // Python -> C++
            bool load(handle src, bool){

                if (!py::isinstance<py::array_t<double>>(src))
                    return false;

                auto arr = py::array_t<double, py::array::c_style | py::array::forcecast>::ensure(src);
                if (!arr)
                    return false;

                if (arr.ndim() != 2 || (arr.shape(1) != 3 && arr.shape(1) != 2))
                    return false;

                bool size3 = true;
                if(arr.shape(1) == 2)
                    size3 = false;
                

                auto buf = arr.unchecked<2>();
                int rows = buf.shape(0);

                value.clear();
                value.reserve(rows);

                for (int i=0; i<rows; i++) {
                    finelc::Node_ptr node = std::make_shared<finelc::Node>();
                    node->x = buf(i, 0);
                    node->y = buf(i, 1);
                    if(size3){
                        node->z = buf(i, 2);
                    }else{
                        node->z = 0;
                    }
                    
                    value.push_back(std::move(node));
                }

                return true;
            }

            // C++ -> Python
            static handle cast(const finelc::VectorNodes& src, return_value_policy policy, handle){

                int size = src.size();

                py::array_t<double> arr({size, 3});
                auto buf = arr.mutable_unchecked<2>();

                for(int i=0; i<size; i++){
                    buf(i,0) = src[i]->x;
                    buf(i,1) = src[i]->y;
                    buf(i,2) = src[i]->z;
                }
                return arr.release();
            }

    };

    using VectorPts = std::vector<finelc::Point>;

    template <> struct type_caster<VectorPts> {

        public:

            PYBIND11_TYPE_CASTER(VectorPts, _("numpy.ndarray[float64, shape=(n,3)]"));

            // Python -> C++
            bool load(handle src, bool){
                return false;
            }

            // C++ -> Python
            static handle cast(const VectorPts& src,return_value_policy policy, handle){

                int size = src.size();

                py::array_t<double> arr({size, 3});
                auto buf = arr.mutable_unchecked<2>();

                for(int i=0; i<size; i++){
                    buf(i,0) = src[i].x;
                    buf(i,1) = src[i].y;
                    buf(i,2) = src[i].z;
                }
                return arr.release();
            }
    };

    using GridData = std::vector<std::pair<finelc::Point,double>>;

    template <> struct type_caster<GridData> {

        public:

            PYBIND11_TYPE_CASTER(GridData, _("Tuple[numpy.ndarray[float64, shape=(n,3)], numpy.ndarray[float64, shape=(n,)]]"));

            // Python -> C++
            bool load(handle src, bool){
                return false;
            }

            // C++ -> Python
            static handle cast(const GridData& src,return_value_policy policy, handle){

                int size = src.size();

                py::array_t<double> arr_loc({size, 3});
                auto buf_loc = arr_loc.mutable_unchecked<2>();

                py::array_t<double> arr_val(size);
                auto buf_val = arr_val.mutable_unchecked<1>();

                for(int i=0; i<size; i++){
                    buf_loc(i,0) = src[i].first.x;
                    buf_loc(i,1) = src[i].first.y;
                    buf_loc(i,2) = src[i].first.z;
                    buf_val(i) = src[i].second;
                }
                return py::make_tuple(arr_loc.release(), arr_val.release()).release();
            }
    };


    
    
}} // namespace pybind11::detail


