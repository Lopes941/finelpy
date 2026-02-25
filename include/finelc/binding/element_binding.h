#pragma once

#include <finelc/matrix.h>

#include <finelc/elements/element.h>
#include <finelc/element_geometry/element_geometry.h>

#include <finelc/binding/matrix_binding.h>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h> 
#include <pybind11/stl.h> 

#include <vector>
#include <memory>

using namespace finelc;
namespace py = pybind11;

class ElementShapeTrampoline: public IElementShape{

    public:

        ~ElementShapeTrampoline() override =default;

        ShapeType shape()const override{
             PYBIND11_OVERLOAD_PURE(
                ShapeType,
                IElementShape,
                shape,
            );
        }

        IntegrationGeometry integration_domain()const override{
            PYBIND11_OVERLOAD_PURE(
                IntegrationGeometry,
                IElementShape,
                integration_domain
            );
        }

        int number_of_dimensions()const override{
            PYBIND11_OVERLOAD_PURE(
                int,
                IElementShape,
                number_of_dimensions,
            );
        }

        int shape_order()const override{
            PYBIND11_OVERLOAD_PURE(
                int,
                IElementShape,
                shape_order,
            );
        }

        int number_of_nodes()const override{
            PYBIND11_OVERLOAD_PURE(
                int,
                IElementShape,
                number_of_nodes,
            );
        }

        int number_of_edges() const override{
            PYBIND11_OVERLOAD_PURE(
                int,
                IElementShape,
                number_of_edges,
            );
        }

        int number_of_faces() const override{
            PYBIND11_OVERLOAD_PURE(
                int,
                IElementShape,
                number_of_faces,
            );
        }

        std::vector<Line_ptr> edges(const VectorNodes& element_nodes)const override{
            PYBIND11_OVERLOAD_PURE(
                std::vector<Line_ptr>,
                IElementShape,
                edges,
                element_nodes
            );
        }

        std::vector<IArea_ptr> surfaces(const VectorNodes& element_nodes)const override{
            PYBIND11_OVERLOAD_PURE(
                std::vector<IArea_ptr>,
                IElementShape,
                surfaces,
                element_nodes
            );
        }

        IGeometry_ptr geometry(const VectorNodes& element_nodes)const override{
            PYBIND11_OVERLOAD_PURE(
                IGeometry_ptr,
                IElementShape,
                geometry,
                element_nodes
            );
        }

        Matrix J(const VectorNodes& element_nodes,
                const Vector& loc=default_zero_vec(3)) const override {

            PYBIND11_OVERLOAD_PURE(
                Matrix,
                IElementShape, 
                J, 
                element_nodes, loc
            );
        }

        double detJ(const VectorNodes& element_nodes,
                const Vector& loc=default_zero_vec(3)) const override {

            PYBIND11_OVERLOAD(
                double,
                IElementShape, 
                detJ, 
                element_nodes, loc
            );
        }

        Vector N(const Vector& loc) const override {
            PYBIND11_OVERLOAD_PURE(
                Vector,
                IElementShape, 
                N, 
                loc
            );
        }

        Matrix dNdxi(const Vector& loc) const override {

            PYBIND11_OVERLOAD_PURE(
                Matrix,
                IElementShape, 
                dNdxi, 
                loc
            );
        }

        Matrix dNdx(const VectorNodes& element_nodes,
                const Vector& loc) const override {

            PYBIND11_OVERLOAD(
                Matrix,
                IElementShape, 
                dNdx, 
                element_nodes, loc
            );
        }


};


class ElementTrampoline: public IElement{

    public:

        ~ElementTrampoline() override =default;

        IElement_ptr copy(bool same_matrix) override{
            PYBIND11_OVERLOAD_PURE(
                IElement_ptr,
                IElement,
                copy,
                same_matrix
            );
        }

        Vector local_to_global(const Vector& loc)const override{
            PYBIND11_OVERLOAD(
                Vector,
                IElement,
                local_to_global,
                loc
            );
        }

        /**********************SHAPE METHODS****************************/
        ShapeType get_shape() const override{
            PYBIND11_OVERLOAD_PURE(
                ShapeType,
                IElement,
                get_shape,
            );
        }

        int number_of_nodes() const override{
            PYBIND11_OVERLOAD_PURE(
                int,
                IElement,
                number_of_nodes,
            );
        }

        int number_of_edges() const override{
            PYBIND11_OVERLOAD_PURE(
                int,
                IElement,
                number_of_edges,
            );
        }

        int number_of_surfaces() const override{
            PYBIND11_OVERLOAD_PURE(
                int,
                IElement,
                number_of_surfaces,
            );
        }

        int number_of_dimensions() const override{
            PYBIND11_OVERLOAD_PURE(
                int,
                IElement,
                number_of_dimensions,
            );
        }

        IntegrationGeometry get_integration_domain() const override{
            PYBIND11_OVERLOAD_PURE(
                IntegrationGeometry,
                IElement,
                get_integration_domain,
            );
        }

        Matrix J(OptionalVector loc) const override{
            PYBIND11_OVERLOAD_PURE(
                Matrix,
                IElement,
                J,
                loc
            );
        }

        double detJ(OptionalVector loc) const override{
            PYBIND11_OVERLOAD_PURE(
                double,
                IElement,
                detJ,
                loc
            );
        }

        std::vector<Line_ptr> edges()const override{
            PYBIND11_OVERLOAD_PURE(
                std::vector<Line_ptr>,
                IElement,
                edges,
            );
        }

        std::vector<IArea_ptr> surfaces()const override{
            PYBIND11_OVERLOAD_PURE(
                std::vector<IArea_ptr>,
                IElement,
                surfaces,
            );
        }

        IGeometry_ptr geometry()const override{
            PYBIND11_OVERLOAD_PURE(
                IGeometry_ptr,
                IElement,
                geometry,
            );
        }

        Vector N_shape(const Vector& loc) const override{
            PYBIND11_OVERLOAD_PURE(
                Vector,
                IElement,
                N_shape,
                loc
            );
        }

        Matrix dNdxi_shape(const Vector& loc) const override{
            PYBIND11_OVERLOAD_PURE(
                Matrix,
                IElement,
                dNdxi_shape,
                loc
            );
        }

        Matrix dNdx_shape(const Vector& loc) const override{
            PYBIND11_OVERLOAD_PURE(
                Matrix,
                IElement,
                dNdx_shape,
                loc
            );
        }

        /********************CONSTITUTIVE METHODS************************/

        ConstitutiveType get_constitutive_model() const override{
            PYBIND11_OVERLOAD_PURE(
                ConstitutiveType,
                IElement,
                get_constitutive_model,
            );
        }

        bool linear_material() const override{
            PYBIND11_OVERLOAD_PURE(
                bool,
                IElement,
                linear_material,
            );
        }

        double get_property(const MaterialProperties& prop) const override{
            PYBIND11_OVERLOAD_PURE(
                double,
                IElement,
                get_property,
                prop
            );
        }

        Matrix D(OptionalVector ue, OptionalVector loc) const override{
            PYBIND11_OVERLOAD_PURE(
                Matrix,
                IElement,
                D,
                ue, loc
            );
        }

        /********************PHYSICS METHODS************************/

        ModelType get_model() const override{
            PYBIND11_OVERLOAD_PURE(
                ModelType,
                IElement,
                get_model,
            );
        }

        std::vector<DOFType> dofs() const override{
            PYBIND11_OVERLOAD_PURE(
                std::vector<DOFType>,
                IElement,
                dofs,
            );
        }
        
        int dofs_per_node() const override{
            PYBIND11_OVERLOAD_PURE(
                int,
                IElement,
                dofs_per_node,
            );
        }
        
        bool linear_physics()const override{
            PYBIND11_OVERLOAD_PURE(
                bool,
                IElement,
                linear_physics,
            );
        }
        

        Matrix N(const Vector& loc, OptionalVector ue) const override{
            PYBIND11_OVERLOAD_PURE(
                Matrix,
                IElement,
                N,
                loc, ue
            );
        }
        
        Matrix dNdx(const Vector& loc, OptionalVector ue) const override{
            PYBIND11_OVERLOAD_PURE(
                Matrix,
                IElement,
                dNdx,
                loc, ue
            );
        }
        
        Matrix B(const Vector& loc, OptionalVector ue)const override{
            PYBIND11_OVERLOAD_PURE(
                Matrix,
                IElement,
                B,
                loc, ue
            );
        }


        /************************MATRIX METHODS*****************************/
        const Matrix& Ke(OptionalVector ue) override{
            PYBIND11_OVERLOAD_PURE(
                const Matrix&,
                IElement,
                Ke,
                ue
            );
        }
        const Matrix& Me(OptionalVector ue) override{
            PYBIND11_OVERLOAD_PURE(
                const Matrix&,
                IElement,
                Me,
                ue
            );
        }

        /*********************RESULT ACCESS METHODS**************************/


            // ELASTICITY
            bool supports_strain()const override{
                PYBIND11_OVERLOAD_PURE(
                    bool,
                    IElement,
                    supports_strain,
                );
            }

            Vector get_strain(const Vector& loc, const Vector& ue)const override{
                PYBIND11_OVERLOAD_PURE(
                    Vector,
                    IElement,
                    get_strain,
                    loc, ue
                );
            }

            bool supports_stress()const override{
                PYBIND11_OVERLOAD_PURE(
                    bool,
                    IElement,
                    supports_stress,
                );
            }

            Vector get_stress(const Vector& loc, const Vector& ue)const override{
                PYBIND11_OVERLOAD_PURE(
                    Vector,
                    IElement,
                    get_stress,
                    loc, ue
                );
            }


            bool supports_NX()const override{
                PYBIND11_OVERLOAD_PURE(
                    bool,
                    IElement,
                    supports_NX,
                );
            }

            Vector get_NX(const Vector& loc, const Vector& ue)const override{
                PYBIND11_OVERLOAD_PURE(
                    Vector,
                    IElement,
                    get_NX,
                    loc, ue
                );
            }

            bool supports_MZ()const override{
                PYBIND11_OVERLOAD_PURE(
                    bool,
                    IElement,
                    supports_MZ,
                );
            }

            Vector get_MZ(const Vector& loc, const Vector& ue)const override{
                PYBIND11_OVERLOAD_PURE(
                    Vector,
                    IElement,
                    get_MZ,
                    loc, ue
                );
            }

            bool supports_VY()const override{
                PYBIND11_OVERLOAD_PURE(
                    bool,
                    IElement,
                    supports_VY,
                );
            }

            Vector get_VY(const Vector& loc, const Vector& ue)const override{
                PYBIND11_OVERLOAD_PURE(
                    Vector,
                    IElement,
                    get_VY,
                    loc, ue
                );
            }

};
