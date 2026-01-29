#pragma once

#include <finelc/matrix.h>
#include <finelc/enumerations.h>

#include <finelc/geometry/geometry.h>

#include <finelc/elements/integration.h>
#include <finelc/elements/element_matrix.h>

#include <finelc/element_geometry/element_geometry.h>
#include <finelc/element_physics/element_physics.h>
#include <finelc/material/constitutive.h>

#include <pybind11/pybind11.h>

#include <iostream>
#include <stdexcept>
#include <vector>
#include <functional>
#include <optional>
#include <variant>

namespace finelc{

    /**
     * @class IElement
     * 
     * @brief Interface for finite elements
     * 
     * This class defines the interface for finite elements in the FinelC library.
     * 
     * It provides methods for setting up the element, accessing shape functions,
     * material properties, physics models, integration points, and elemental matrices.
     * It also includes methods for accessing results such as strain and stress.
     */
    class IElement{

        private:

            int num_int_pts; // Number of integration points
            std::vector<int> node_numbering; // Node numbering for the element
            const VectorNodes* mesh_nodes=nullptr; // Pointer to the mesh nodes associated with the element


        protected:

            /**
             * @brief Get the nodes associated with the mesh object that has this element.
             * 
             * @return const VectorNodes& Reference to the vector of node pointers.
             */
            const VectorNodes& get_mesh_nodes()const;

            /**
             * @brief Check if the provided node numbering is valid.
             * 
             * Checks if the size of the provided node numbering matches the number of nodes
             * of the element. Throws a runtime error if they do not match.
             * 
             * @param node_numbering_ The vector of node indices to check.
             * 
             * @throws std::runtime_error if the size of node_numbering_ does not match number_of_nodes().
             */
            void check_node_numbering(const std::vector<int>& node_numbering_)const;

        public:

            /**
             * @brief Pointer to the Python object associated with this element.
             * 
             * This member is used for class definitions in pybind11 bindings.
             */
            std::unique_ptr<pybind11::object> py_self;

            virtual ~IElement() = default;


            using IElement_ptr = std::shared_ptr<IElement>;

            /**
             * @brief Create a copy of the element.
             * 
             * @param same_matrix If true, the copied element will share the same elemental matrices
             *                    as the original element (only if both elements are linear).
             * @return IElement_ptr A shared pointer to the copied element.
             */
            virtual IElement_ptr copy(bool same_matrix=false)=0;

            /**
             * @brief Copy the internal data to another element.
             * 
             * @param other The element to copy the data to.
             * 
             * @throws std::runtime_error if the provided element pointer is null.
             */
            void copy_to(IElement_ptr other)const{
                if (!other) {
                    throw std::runtime_error("copy_to received null IElement_ptr");
                }
                other->node_numbering   = node_numbering;
                other->mesh_nodes       = mesh_nodes;
            }

            /**********************SET UP METHODS****************************/

            /**
             * @brief Set the node numbering for the element.
             * 
             * @param node_numbering_ The vector of node indices defining the node numbering.
             * @param mesh_nodes_ The vector of node pointers representing the mesh nodes.
             * 
             * @throws std::runtime_error if the size of node_numbering_ does not match number_of_nodes().
             */
            void set_node_numbering(const std::vector<int>& node_numbering_, 
                                    const VectorNodes& mesh_nodes_);

                   
            /**
             * @brief Set the node numbering for the element.
             * 
             * @param node_numbering_ The vector of node indices defining the node numbering.
             * 
             * @throws std::runtime_error if the size of node_numbering_ does not match number_of_nodes().
             */
            void set_node_numbering(const std::vector<int>& node_numbering_);

            /**
             * @brief Get the node numbering of the element.
             * 
             * @return const std::vector<int>& Reference to the vector of node indices defining the node numbering.
             * 
             * @throws std::runtime_error if the node numbering has not been set.
             */
            const std::vector<int>& get_node_numbering()const;

            /**
             * @brief Get the nodes of the element.
             * 
             * @return VectorNodes A vector of shared pointers to the nodes of the element.
             */
            VectorNodes get_nodes()const;

            /**
             * @brief Check if the element is linear.
             * 
             * This method checks if both the shape and physics of the element are linear.
             * This is useful for determining if certain optimizations can be applied.
             * 
             * @return true if the element is linear, false otherwise.
             */
            bool is_linear()const;

            /**
             * @brief Get the size of the displacement vector for the element.
             * 
             * This method calculates the size of the displacement vector based on the
             * number of nodes and the degrees of freedom per node.
             * 
             * @return int The size of the displacement vector.
             */
            int displacement_size()const;

            /*********************INTERPOLATION METHODS**************************/

            /**
             * @brief Interpolate nodal values at a given location.
             * 
             * This method interpolates the nodal values at the specified local coordinates
             * using the shape functions of the element.
             * 
             * @param loc The local coordinates where the interpolation is to be performed.
             * @param ue The vector of nodal values to be interpolated.
             * 
             * @return double The interpolated value at the specified location.
             */
            double interpolate_nodal_values(const Vector& loc, 
                                            const Vector& ue)const;

            /**
             * @brief Interpolate result vector at a given location.
             * 
             * This method interpolates the result vector at the specified local coordinates
             * using the shape functions of the element.
             * 
             * @param loc The local coordinates where the interpolation is to be performed.
             * @param ue The vector of nodal values to be interpolated.
             * @return Vector The interpolated result vector at the specified location.
             */
            Vector interpolate_result(const Vector& loc, 
                                    const Vector& ue)const;

            /**
             * @brief Calculate the geometric center of the element.
             * 
             * This method computes the geometric center (centroid) of the element
             * based on the coordinates of its nodes.
             * 
             * @return Point The geometric center of the element.
             */
            Point geometric_center()const;

            /*********************COORDINATE TRANSFORMATION METHODS**************************/
            
            /**
             * @brief Transform local coordinates to global coordinates.
             * 
             * @param loc The local coordinates to be transformed.
             * 
             * @return Vector The corresponding global coordinates.
             */
            virtual Vector local_to_global(const Vector& loc)const;

            /**
             * @brief Transform local coordinates to global coordinates.
             * 
             * @param loc The local coordinates to be transformed.
             * 
             * @return Point The corresponding global coordinates.
             */
            Point local_to_global(const Point& loc)const;

            /**********************SHAPE METHODS****************************/

            /**
             * @brief Get the shape type of the element.
             * 
             * @return ShapeType The shape type of the element.
             */
            virtual ShapeType get_shape() const=0;

            /**
             * @brief Get the number of nodes of the element.
             * 
             * @return int The number of nodes of the element.
             */
            virtual int number_of_nodes() const=0;

            /**
             * @brief Get the number of vertices of the element.
             * 
             * @return int The number of vertices of the element.
             */
            virtual int number_of_vertices() const=0;

            /**
             * @brief Get the number of dimensions of the element.
             * 
             * @return int The number of dimensions of the element.
             */
            virtual int number_of_dimensions() const=0;

            /**
             * @brief Get the integration geometry of the element.
             * 
             * @return IntegrationGeometry The integration geometry of the element.
             */
            virtual IntegrationGeometry get_integration_domain()const=0;

            /**
             * @brief Compute the Jacobian matrix at a given location.
             * 
             * @param loc The local coordinates where the Jacobian is to be computed.
             * 
             * @return Matrix The Jacobian matrix at the specified location.
             */
            virtual Matrix J(OptionalVector loc)const=0;

            /**
             * @brief Compute the determinant of the Jacobian at a given location.
             * 
             * @param loc The local coordinates where the determinant is to be computed.
             * 
             * @return double The determinant of the Jacobian at the specified location.
             */
            virtual double detJ(OptionalVector loc)const=0;


            /**
             * @brief Get the edges of the element.
             * 
             * @return std::vector<Line_ptr> A vector of shared pointers to the edges of the element.
             */
            virtual std::vector<Line_ptr> edges()const=0;

            /**
             * @brief Get the surfaces of the element.
             * 
             * @return std::vector<IArea_ptr> A vector of shared pointers to the surfaces of the element.
             */
            virtual std::vector<IArea_ptr> surfaces()const=0;


            /**
             * @brief Get the shape functions at a given location.
             * 
             * This method returns the shape functions evaluated at the specified local coordinates. These functions are related to the geometry of the element.
             * 
             * @param loc The local coordinates where the shape functions are to be evaluated.
             * 
             * @return Vector The shape functions evaluated at the specified location.
             */
            virtual Vector N_shape(const Vector& loc) const=0;

            /**
             * @brief Get the derivatives of the shape functions with respect to local coordinates at a given location.
             * 
             * This method returns the derivatives of the shape functions with respect to the local coordinates, evaluated at the specified location. These functions are related to the geometry of the element.
             * 
             * @param loc The local coordinates where the derivatives are to be evaluated.
             * 
             * @return Matrix The derivatives of the shape functions evaluated at the specified location.
             */
            virtual Matrix dNdxi_shape(const Vector& loc) const=0;

            /**
             * @brief Get the derivatives of the shape functions with respect to global coordinates at a given location.
             * 
             * This method returns the derivatives of the shape functions with respect to the global coordinates, evaluated at the specified location. These functions are related to the geometry of the element.
             * 
             * @param loc The local coordinates where the derivatives are to be evaluated.
             * 
             * @return Matrix The derivatives of the shape functions evaluated at the specified location.
             */
            virtual Matrix dNdx_shape(const Vector& loc) const=0;

            
            /******************CONSTITUTIVE METHODS************************/

            /**
             * @brief Get the constitutive model type of the element.
             * 
             * @return ConstitutiveType The constitutive model type of the element.
             */
            virtual ConstitutiveType get_constitutive_model() const=0;

            /**
             * @brief Check if the material is linear.
             * 
             * @return true if the material is linear, false otherwise.
             */
            virtual bool linear_material()const=0;

            /**
             * @brief Get a material property of the element.
             * 
             * @param prop The material property to retrieve.
             * @return double The value of the requested material property.
             */
            virtual double get_property(const MaterialProperties& prop) const=0;

            /**
             * @brief Get the constitutive matrix at a given location.
             * 
             * @param loc The local coordinates where the constitutive matrix is to be evaluated (optional).
             * @param ue The vector of nodal values (optional). Used for nonlinear materials.
             * 
             * @return Matrix The constitutive matrix at the specified location.
             */
            virtual Matrix D(OptionalVector loc, OptionalVector ue)const=0;


            /*********************PHYSICS METHODS**************************/

            /**
             * @brief Get the physics model type of the element.
             * 
             * @return ModelType The physics model type of the element.
             */
            virtual ModelType get_model() const=0;

            /**
             * @brief Get the degrees of freedom of the element.
             * 
             * @return std::vector<DOFType> A vector of degrees of freedom types for the element.
             */
            virtual std::vector<DOFType> dofs() const=0;

            /**
             * @brief Get the number of degrees of freedom per node.
             * 
             * @return int The number of degrees of freedom per node.
             */
            virtual int dofs_per_node() const=0;

            /**
             * @brief Check if the physics is linear.
             * 
             * @return true if the physics is linear, false otherwise.
             */
            virtual bool linear_physics()const=0;

            /**
             * @brief Get the interpolation functions for the physics at a given location.
             * 
             * This method returns the interpolation functions evaluated at the specified local coordinates. These functions are related to the physics of the element.
             * 
             * @param loc The local coordinates where the interpolation functions are to be evaluated.
             * @param ue The vector of nodal values (optional). Used for nonlinear physics.
             * 
             * @return Matrix The interpolation functions evaluated at the specified location.
             */
            virtual Matrix N(const Vector& loc, OptionalVector ue) const=0;

            /**
             * @brief Get the derivatives of the interpolation functions with respect to global coordinates at a given location.
             * 
             * This method returns the derivatives of the interpolation functions with respect to the global coordinates, evaluated at the specified location. These functions are related to the physics of the element.
             * 
             * @param loc The local coordinates where the derivatives are to be evaluated.
             * @param ue The vector of nodal values (optional). Used for nonlinear physics.
             * 
             * @return Matrix The derivatives of the interpolation functions evaluated at the specified location.
             */
            virtual Matrix dNdx(const Vector& loc, OptionalVector ue) const=0;

            /**
             * @brief Get the strain-displacement matrix at a given location.
             * 
             * @param loc The local coordinates where the strain-displacement matrix is to be evaluated.
             * @param ue The vector of nodal values (optional). Used for nonlinear physics.
             * 
             * @return Matrix The strain-displacement matrix at the specified location.
             */
            virtual Matrix B(const Vector& loc, OptionalVector ue)const=0;


            /************************INTEGRATION METHODS*****************************/

            /**
             * @brief Set the number of integration points for the element.
             * 
             * @param number_points The number of integration points to set.
             */
            void set_number_integration_points(int number_points);

            /**
             * @brief Get the number of integration points for the element.
             * 
             * @return int The number of integration points.
             */
            int get_number_integration_points()const;

            /**
             * @brief Get the integration points and weights for the element.
             * 
             * @param number_points The number of integration points to use (optional). If 0, number of integration points set up by the element is used.
             * 
             * @return std::vector<PointWeight> A vector of PointWeight structures containing the integration points and their corresponding weights.
             */
            std::vector<PointWeight> integration_pair(int number_points=0)const;

            /**
             * @brief Get the integration points for the element.
             * 
             * @param number_points The number of integration points to use (optional). If 0, number of integration points set up by the element is used.
             * 
             * @return std::vector<Point> A vector of integration points.
             */
            std::vector<Point> integration_points(int number_points=0)const;

            /**
             * @brief Get the integration weights for the element.
             * 
             * @param number_points The number of integration points to use (optional). If 0, number of integration points set up by the element is used.
             * 
             * @return std::vector<double> A vector of integration weights.
             */
            std::vector<double> integration_weights(int number_points=0)const;

            /************************MATRIX METHODS*****************************/

            /**
             * @brief Get the elemental stiffness matrix.
             * 
             * @param ue The vector of nodal values (optional). Used for nonlinear physics/materials.
             * 
             * @return const Matrix& The elemental stiffness matrix.
             */
            virtual const Matrix& Ke(OptionalVector ue)=0;

            /**
             * @brief Get the elemental mass matrix.
             * 
             * @param ue The vector of nodal values (optional). Used for nonlinear physics/materials.
             * 
             * @return const Matrix& The elemental mass matrix.
             */
            virtual const Matrix& Me(OptionalVector ue)=0;


            /*********************RESULT ACCESS METHODS**************************/

            /**
             * @brief Check if the element has a specific degree of freedom.
             * 
             * @param type The degree of freedom type to check.
             * 
             * @return true if the element has the specified degree of freedom, false otherwise.
             */
            bool has_dof(DOFType type)const;


            /**
             * @brief Check if the element supports displacement degrees of freedom.
             * 
             * @return true if the element supports displacement degrees of freedom, false otherwise.
             * 
             * @see has_dof(DOFType type)
             */
            bool supports_displacement()const{return (has_dof(DOFType::UX) ||
                                                        has_dof(DOFType::UY) ||
                                                        has_dof(DOFType::UZ));}


            
            // ELASTICITY RESULTS

            /**
             * @brief Check if the element supports strain computation.
             * 
             * @return true if the element supports strain computation, false otherwise.
             */
            virtual bool supports_strain()const=0;

            /**
             * @brief Get the strain at a given location.
             * 
             * @param loc The local coordinates where the strain is to be computed.
             * @param ue The vector of nodal values.
             * 
             * @return Vector The strain at the specified location.
             */
            virtual Vector get_strain(const Vector& loc, const Vector& ue)const=0;

            /**
             * @brief Check if the element supports stress computation.
             * 
             * @return true if the element supports stress computation, false otherwise.
             */
            virtual bool supports_stress()const=0;

            /**
             * @brief Get the stress at a given location.
             * 
             * @param loc The local coordinates where the stress is to be computed.
             * @param ue The vector of nodal values.
             * 
             * @return Vector The stress at the specified location.
             */
            virtual Vector get_stress(const Vector& loc, const Vector& ue)const=0;

            /**
             * @brief Check if the element supports axial force result.
             * 
             * @return true if the element supports axial force result, false otherwise.
             */
            virtual bool supports_NX()const=0;

            /**
             * @brief Get the axial force at a given location.
             * 
             * @param loc The local coordinates where the axial force is to be computed.
             * @param ue The vector of nodal values.
             * 
             * @return Vector The axial force at the specified location.
             */
            virtual Vector get_NX(const Vector& loc, const Vector& ue)const=0;


            /**
             * @brief Check if the element supports shear force result in Y direction.
             * 
             * @return true if the element supports shear force result in Y direction, false otherwise.
             */
            virtual bool supports_VY()const=0;

            /**
             * @brief Get the shear force in Y direction at a given location.
             * 
             * @param loc The local coordinates where the shear force is to be computed.
             * @param ue The vector of nodal values.
             * 
             * @return Vector The shear force in Y direction at the specified location.
             */
            virtual Vector get_VY(const Vector& loc, const Vector& ue)const=0;


            /**
             * @brief Check if the element supports bending moment in Z direction.
             * 
             * @return true if the element supports bending moment in Z direction, false otherwise.
             */
            virtual bool supports_MZ()const=0;

            /**
             * @brief Get the bending moment in Z direction at a given location.
             * 
             * @param loc The local coordinates where the bending moment is to be computed.
             * @param ue The vector of nodal values.
             * 
             * @return Vector The bending moment in Z direction at the specified location.
             */
            virtual Vector get_MZ(const Vector& loc, const Vector& ue)const=0;

            

    };

    /**
     * @class Element
     * 
     * @brief Concrete implementation of a finite element.
     * 
     * This class implements the IElement interface and represents a finite element
     * in the FinelC library. It combines shape, physics, and material models
     * to provide a complete representation of an element.
     * 
     * It provides methods for setting up the element, accessing shape functions,
     * material properties, physics models, integration points, and elemental matrices.
     * It also includes methods for accessing results such as strain and stress.
     */
    class Element: public IElement{

        private:

            IElementShape_ptr shape; // Pointer to the shape model of the element
            IElementPhysics_ptr physics; // Pointer to the physics model of the element
            IConstitutiveModel_ptr material; //  Pointer to the constitutive model of the element
            ElementalMatrices_ptr matrices; // Pointer to the elemental matrices of the element

            /**
             * @brief Check if the matrices can be shared between elements.
             * 
             * @param same_matrix Indicates whether the matrices are intended to be shared.
             * 
             * @return true if the matrices can be shared, false otherwise.
             */
            bool check_matrix_similarity(bool same_matrix){
                return same_matrix &&
                        is_linear();
            }

        public:

            Element(IElementShape_ptr shape_,
                    IElementPhysics_ptr physics_,
                    IConstitutiveModel_ptr material_):
                shape(shape_),
                physics(physics_),
                material(material_)
                {
                    set_number_integration_points(std::ceil((2.*shape_->shape_order()+1.)/2.));
                    matrices = std::make_shared<ElementalMatrices>();
                }

            using Element_ptr = std::shared_ptr<Element>;

            /**
             * @brief Create a copy of the element.
             * 
             * @param same_matrix If true, the copied element will share the same elemental matrices
             *                    as the original element (only if both elements are linear).
             * 
             * @return IElement_ptr A shared pointer to the copied element.
             */
            IElement_ptr copy(bool same_matrix=false) override{
                Element_ptr new_element = std::make_shared<Element>(shape,physics,material);
                new_element->set_node_numbering(get_node_numbering(),get_mesh_nodes());
                new_element->set_number_integration_points(get_number_integration_points());

                if(check_matrix_similarity(same_matrix)){
                    new_element->matrices = matrices;
                }
                return new_element;
            }

            /**********************SHAPE METHODS****************************/
            /**
             * @brief Get the shape type of the element.
             * 
             * @return ShapeType The shape type of the element.
             */
            ShapeType get_shape() const override{return shape->shape();}     
            
            /**
             * @brief Get the number of nodes of the element.
             * 
             * @return int The number of nodes of the element.
             */
            int number_of_nodes() const override{return shape->number_of_nodes();}

            /**
             * @brief Get the number of vertices of the element.
             * 
             * The number of vertices may be different from the number of nodes.
             * For example, both QUAD4 and QUAD9 have 4 vertices. But the first 
             * has 4 nodes and the second, 9 nodes.
             * 
             * @return int The number of vertices of the element.
             */
            int number_of_vertices() const override{return shape->number_of_vertices();}

            /**
             * @brief Get the number of dimensions of the element.
             * 
             * @return int The number of dimensions of the element.
             */
            int number_of_dimensions() const override{return shape->number_of_dimensions();}

            /**
             * @brief Get the integration geometry of the element.
             * 
             * @return IntegrationGeometry The integration geometry of the element.
             */
            IntegrationGeometry get_integration_domain()const override{return shape->integration_domain();}

            /**
             * @brief Compute the Jacobian matrix at a given location.
             * 
             * @param loc The local coordinates where the Jacobian is to be computed.
             * 
             * @return Matrix The Jacobian matrix at the specified location.
             */
            Matrix J(OptionalVector loc)const override{
                return shape->J(get_nodes(), loc? loc->get():default_zero_vec(3) );
            }

            /**
             * @brief Compute the determinant of the Jacobian at a given location.
             * 
             * @param loc The local coordinates where the determinant is to be computed.
             * 
             * @return double The determinant of the Jacobian at the specified location.
             */
            double detJ(OptionalVector loc)const override{
                return shape->detJ(get_nodes(), loc? loc->get():default_zero_vec(3) );
            }

            /**
             * @brief Get the edges of the element.
             * 
             * @return std::vector<Line_ptr> A vector of shared pointers to the edges of the element.
             */
            virtual std::vector<Line_ptr> edges()const override{return shape->edges(get_nodes());}

            /**
             * @brief Get the surfaces of the element.
             * 
             * @return std::vector<IArea_ptr> A vector of shared pointers to the surfaces of the element.
             */
            virtual std::vector<IArea_ptr> surfaces()const override{return shape->surfaces(get_nodes());}

            /**
             * @brief Get the shape functions at a given location.
             * 
             * This method returns the shape functions evaluated at the specified local coordinates. These functions are related to the geometry of the element.
             * 
             * @param loc The local coordinates where the shape functions are to be evaluated.
             * 
             * @return Vector The shape functions evaluated at the specified location.
             */
            Vector N_shape(const Vector& loc) const override{return shape->N(loc);}

            /**
             * @brief Get the derivatives of the shape functions with respect to local coordinates at a given location.
             * 
             * This method returns the derivatives of the shape functions with respect to the local coordinates, evaluated at the specified location. These functions are related to the geometry of the element.
             * 
             * @param loc The local coordinates where the derivatives are to be evaluated.
             * 
             * @return Matrix The derivatives of the shape functions evaluated at the specified location.
             */
            Matrix dNdxi_shape(const Vector& loc) const override{return shape->dNdxi(loc);}

            /**
             * @brief Get the derivatives of the shape functions with respect to global coordinates at a given location.
             * 
             * This method returns the derivatives of the shape functions with respect to the global coordinates, evaluated at the specified location. These functions are related to the geometry of the element.
             * 
             * @param loc The local coordinates where the derivatives are to be evaluated.
             * 
             * @return Matrix The derivatives of the shape functions evaluated at the specified location.
             */
            Matrix dNdx_shape(const Vector& loc) const override{
                return shape->dNdx(get_nodes(), loc);
            }

            

            /******************CONSTITUTIVE METHODS************************/

            /**
             * @brief Get the constitutive model type of the element.
             * 
             * @return ConstitutiveType The constitutive model type of the element.
             */
            ConstitutiveType get_constitutive_model() const override{ return material->contitutive_model();}

            /**
             * @brief Check if the material is linear.
             * 
             * @return true if the material is linear, false otherwise.
             */
            bool linear_material()const override{return material->is_linear();}

            /**
             * @brief Get a material property of the element.
             * 
             * @param prop The material property to retrieve.
             * @return double The value of the requested material property.
             */
            double get_property(const MaterialProperties& prop) const override{ return material->get_property(prop);}

            /**
             * @brief Get the constitutive matrix at a given location.
             * 
             * @param loc The local coordinates where the constitutive matrix is to be evaluated (optional).
             * @param ue The vector of nodal values (optional). Used for nonlinear materials.
             * 
             * @return Matrix The constitutive matrix at the specified location.
             */
            Matrix D(OptionalVector loc, OptionalVector ue)const override{
                return material->D(
                    loc? loc->get():default_zero_vec(3),
                    ue? ue->get():default_zero_vec(displacement_size()));
            }



            /*********************PHYSICS METHODS**************************/

            /**
             * @brief Get the physics model type of the element.
             * 
             * @return ModelType The physics model type of the element.
             */
            ModelType get_model() const override{ return physics->get_model();}

            /**
             * @brief Get the degrees of freedom of the element.
             * 
             * @return std::vector<DOFType> A vector of degrees of freedom types for the element.
             */
            std::vector<DOFType> dofs() const override{return physics->dofs();}

            /**
             * @brief Get the number of degrees of freedom per node.
             * 
             * @return int The number of degrees of freedom per node.
             */
            int dofs_per_node() const override{return physics->dof_per_node();}

            /**
             * @brief Check if the physics is linear.
             * 
             * @return true if the physics is linear, false otherwise.
             */
            bool linear_physics()const override{return physics->is_linear();}
            

            /**
             * @brief Get the interpolation functions for the physics at a given location.
             * 
             * This method returns the interpolation functions evaluated at the specified local coordinates. These functions are related to the physics of the element.
             * 
             * @param loc The local coordinates where the interpolation functions are to be evaluated.
             * @param ue The vector of nodal values (optional). Used for nonlinear physics.
             * 
             * @return Matrix The interpolation functions evaluated at the specified location.
             */
            Matrix N(const Vector& loc, OptionalVector ue) const override{
                return physics->N(N_shape(loc),loc);
            }

            /**
             * @brief Get the derivatives of the interpolation functions with respect to global coordinates at a given location.
             * 
             * This method returns the derivatives of the interpolation functions with respect to the global coordinates, evaluated at the specified location. These functions are related to the physics of the element.
             * 
             * @param loc The local coordinates where the derivatives are to be evaluated.
             * @param ue The vector of nodal values (optional). Used for nonlinear physics.
             * 
             * @return Matrix The derivatives of the interpolation functions evaluated at the specified location.
             */
            Matrix dNdx(const Vector& loc, OptionalVector ue) const override{
                return physics->dNdx(dNdx_shape(loc),loc);
            }

            /**
             * @brief Get the strain-displacement matrix at a given location.
             * 
             * @param loc The local coordinates where the strain-displacement matrix is to be evaluated.
             * @param ue The vector of nodal values (optional). Used for nonlinear physics.
             * 
             * @return Matrix The strain-displacement matrix at the specified location.
             */
            Matrix B(const Vector& loc,OptionalVector ue)const override{
                return physics->B(dNdx(loc,ue),loc);
            }

            /************************MATRIX METHODS*****************************/

            /**
             * @brief Get the elemental stiffness matrix.
             * 
             * @param ue The vector of nodal values (optional). Used for nonlinear physics/materials.
             * 
             * @return const Matrix& The elemental stiffness matrix.
             */
            const Matrix& Ke(OptionalVector ue) override;

            /**
             * @brief Get the elemental mass matrix.
             * 
             * @param ue The vector of nodal values (optional). Used for nonlinear physics/materials.
             * 
             * @return const Matrix& The elemental mass matrix.
             */
            const Matrix& Me(OptionalVector ue) override;

            /*********************RESULT ACCESS METHODS**************************/

            // ELASTICITY RESULTS

            /**
             * @brief Check if the element supports strain computation.
             * 
             * @return true if the element supports strain computation, false otherwise.
             */
            bool supports_strain()const override{return physics->supports_strain();}

            /**
             * @brief Get the strain at a given location.
             * 
             * @param loc The local coordinates where the strain is to be computed.
             * @param ue The vector of nodal values.
             * 
             * @return Vector The strain at the specified location.
             */
            Vector get_strain(const Vector& loc, const Vector& ue)const override{
                return physics->get_strain(loc,ue,dNdx(loc,ue),D(ue,loc));
            }

            /**
             * @brief Check if the element supports stress computation.
             * 
             * @return true if the element supports stress computation, false otherwise.
             */
            bool supports_stress()const override{return physics->supports_stress();}

            /**
             * @brief Get the stress at a given location.
             * 
             * @param loc The local coordinates where the stress is to be computed.
             * @param ue The vector of nodal values.
             * 
             * @return Vector The stress at the specified location.
             */
            Vector get_stress(const Vector& loc, const Vector& ue)const override{
                return physics->get_stress(loc,ue,dNdx(loc,ue),D(ue,loc));
            }

            /**
             * @brief Check if the element supports axial force result.
             * 
             * @return true if the element supports axial force result, false otherwise.
             */
            bool supports_NX()const override{return physics->supports_stress();}

            /**
             * @brief Get the axial force at a given location.
             * 
             * @param loc The local coordinates where the axial force is to be computed.
             * @param ue The vector of nodal values.
             * 
             * @return Vector The axial force at the specified location.
             */
            Vector get_NX(const Vector& loc, const Vector& ue)const override{
                return physics->get_NX(loc,ue,dNdx(loc,ue),D(ue,loc));
            }

            /**
             * @brief Check if the element supports shear force result in Y direction.
             * 
             * @return true if the element supports shear force result in Y direction, false otherwise.
             */
            bool supports_VY()const override{return physics->supports_stress();}

            /**
             * @brief Get the shear force in Y direction at a given location.
             * 
             * @param loc The local coordinates where the shear force is to be computed.
             * @param ue The vector of nodal values.
             * 
             * @return Vector The shear force in Y direction at the specified location.
             */
            Vector get_VY(const Vector& loc, const Vector& ue)const override{
                return physics->get_VY(loc,ue,dNdx(loc,ue),D(ue,loc));
            }

            /**
             * @brief Check if the element supports bending moment in Z direction.
             * 
             * @return true if the element supports bending moment in Z direction, false otherwise.
             */
            bool supports_MZ()const override{return physics->supports_stress();}

            /**
             * @brief Get the bending moment in Z direction at a given location.
             * 
             * @param loc The local coordinates where the bending moment is to be computed.
             * @param ue The vector of nodal values.
             * 
             * @return Vector The bending moment in Z direction at the specified location.
             */
            Vector get_MZ(const Vector& loc, const Vector& ue)const override{
                return physics->get_MZ(loc,ue,dNdx(loc,ue),D(ue,loc));
            }

            
    };

    /**
     * @brief Type alias for a shared pointer to an IElement.
     */
    using IElement_ptr = std::shared_ptr<IElement>; 

    /**
     * @brief Type alias for a vector of shared pointers to IElement.
     */
    using VectorElements = std::vector<IElement_ptr>;


/**
 * @class ElementBuilder
 * 
 * @brief Builder class for creating finite elements.
 * 
 * This class provides a flexible way to create finite elements by specifying their shape, physics, and constitutive models. It checks for compatibility between the specified models and constructs the appropriate element.
 * 
 * @see IElement, Element, IElementShape, IElementPhysics, IConstitutiveModel
 */
    class ElementBuilder{

        private:

            /**
             * @brief The built element.
             */
            IElement_ptr built_element=nullptr;

            /**
             * @brief The shape model of the element.
             */
            IElementShape_ptr shape;

            /**
             * @brief The physics model of the element.
             */
            IElementPhysics_ptr physics;

            /**
             * @brief The constitutive model of the element.
             */
            IConstitutiveModel_ptr material;

            /**
             * @brief Check compatibility between shape, physics, and constitutive models.
             * 
             * @param shape_type The shape type of the element.
             * @param physics_type The physics model type of the element.
             * @param const_type The constitutive model type of the element.
             */
            void check_compatibility(   ShapeType shape_type,
                                        ModelType physics_type,
                                        ConstitutiveType const_type);

            /**
             * @brief Build the shape model based on the specified shape type.
             * 
             * @param shape_type The shape type of the element.
             * 
             * @return IElementShape_ptr A shared pointer to the built shape model.
             */
            IElementShape_ptr       build_shape(ShapeType shape_type);

            /**
             * @brief Build the physics model based on the specified physics type.
             * 
             * @param physics_type The physics model type of the element.
             * 
             * @return IElementPhysics_ptr A shared pointer to the built physics model.
             */
            IElementPhysics_ptr     build_physics(ModelType physics_type);

            /**
             * @brief Build the constitutive model based on the specified constitutive type and material.
             * 
             * @param const_type The constitutive model type of the element.
             * 
             * @return IConstitutiveModel_ptr A shared pointer to the built constitutive model.
             */
            IConstitutiveModel_ptr  build_constitutive(ConstitutiveType const_type, Material_ptr mat);

            /**
             * @brief Create the element based on the specified shape, physics, and constitutive models.
             */
            void create_element();

        public:

            ElementBuilder( std::variant<ShapeType, IElementShape_ptr> shape_type,
                            std::variant<ModelType, IElementPhysics_ptr> physics_input,
                            std::variant<ConstitutiveType, IConstitutiveModel_ptr> const_input,
                            Material_ptr mat = nullptr);


            ~ElementBuilder()=default;

            /**
             * @brief Build and return the constructed element.
             * 
             * @return IElement_ptr A shared pointer to the constructed element.
             */
            IElement_ptr build();

    };


    /**
     * @brief Create a finite element using the ElementBuilder.
     * 
     * This function provides a convenient way to create a finite element by specifying its shape, physics, and constitutive models. It utilizes the ElementBuilder class to construct the element.
     * 
     * @param shape_input The shape type or shape model of the element.
     * @param physics_input The physics model type or physics model of the element.
     * @param const_input The constitutive model type or constitutive model of the element.
     * @param mat The material to be used for the constitutive model (optional).
     * 
     * @return IElement_ptr A shared pointer to the created finite element.
     */
    inline IElement_ptr create_element( std::variant<ShapeType, IElementShape_ptr> shape_input,
                            std::variant<ModelType, IElementPhysics_ptr> physics_input,
                            std::variant<ConstitutiveType, IConstitutiveModel_ptr> const_input,
                            Material_ptr mat = nullptr){
                                ElementBuilder el_builder(shape_input,physics_input,const_input,mat);
                                return el_builder.build();
                            }


} // namespace finel::elements
