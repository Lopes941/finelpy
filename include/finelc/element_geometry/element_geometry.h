#pragma once


#include <finelc/matrix.h>
#include <finelc/enumerations.h>

#include <finelc/material/material.h>
#include <finelc/geometry/geometry.h>

#include <iostream>
#include <vector>
#include <exception>
#include <memory>
#include <stdexcept>


namespace finelc{

    /**
     * @class IElementShape
     * 
     * @brief Interface for element shape functions and geometry.
     */
    class IElementShape{
         
        public:
            virtual ~IElementShape() = default;

            /**
             * @brief Get the shape type of the element.
             * 
             * @return ShapeType The shape type of the element.
             */
            virtual ShapeType shape()const=0;

            /**
             * @brief Get the number of nodes of the element.
             * 
             * @return int The number of nodes of the element.
             */
            virtual int number_of_nodes()const=0;

            /**
             * @brief Get the number of edges of the element.
             * 
             * The number of vertices may be different from the number of nodes.
             * For example, QUAD9 has 8 edges, but 9 nodes.
             * 
             * @return int The number of edges of the element.
             */
            virtual int number_of_edges()const=0;

            /**
             * @brief Get the number of faces of the element.
             * 
             * @return int The number of faces of the element.
             */
            virtual int number_of_faces()const=0;

            /**
             * @brief Get the number of dimensions of the element.
             * 
             * @return int The number of dimensions of the element.
             */
            virtual int number_of_dimensions()const=0;

            /**
             * @brief Get the integration geometry of the element.
             * 
             * @return IntegrationGeometry The integration geometry of the element.
             */
            virtual IntegrationGeometry integration_domain()const=0;
            
            /**
             * @brief Get the integration order of the element.
             * 
             * @return int The integration order of the element.
             */
            virtual int shape_order()const=0;

            /**
             * @brief Compute the Jacobian matrix at a given location.
             * 
             * @param element_nodes Vector of shared pointers to the nodes of the element.
             * @param loc The local coordinates where the Jacobian is to be computed.
             * 
             * @return Matrix The Jacobian matrix at the specified location.
             */
            virtual Matrix J(const VectorNodes& element_nodes,const Vector& loc)const=0;

            /**
             * @brief Compute the determinant of the Jacobian at a given location.
             * 
             * @param element_nodes Vector of shared pointers to the nodes of the element.
             * @param loc The local coordinates where the determinant is to be computed.
             * 
             * @return double The determinant of the Jacobian at the specified location.
             */
            virtual double detJ(const VectorNodes& element_nodes,const Vector& loc)const{
                return J(element_nodes,loc).det();
            }

            /**
             * @brief Get the edges of the element.
             * 
             * @param element_nodes Vector of shared pointers to the nodes of the element.
             * 
             * @return std::vector<Line_ptr> A vector of shared pointers to the edges of the element.
             */
            virtual std::vector<Line_ptr> edges(const VectorNodes& element_nodes)const=0;

            /**
             * @brief Get the surfaces of the element.
             * 
             * @param element_nodes Vector of shared pointers to the nodes of the element.
             * 
             * @return std::vector<IArea_ptr> A vector of shared pointers to the surfaces of the element.
             */
            virtual std::vector<IArea_ptr> surfaces(const VectorNodes& element_nodes)const=0;

            /**
             * @brief Get geometry of the element.
             * 
             * @param element_nodes Vector of shared pointers to the nodes of the element.
             * 
             * @return IGeometry_ptr The geometry of the element.
             */
            virtual IGeometry_ptr geometry(const VectorNodes& element_nodes)const=0;

            /**
             * @brief Get the shape functions at a given location.
             * 
             * This method returns the shape functions evaluated at the specified local coordinates. These functions are related to the geometry of the element.
             * 
             * @param loc The local coordinates where the shape functions are to be evaluated.
             * 
             * @return Vector The shape functions evaluated at the specified location.
             */
            virtual Vector N(const Vector& loc)const=0;

            /**
             * @brief Get the derivatives of the shape functions with respect to local coordinates at a given location.
             * 
             * This method returns the derivatives of the shape functions with respect to the local coordinates, evaluated at the specified location. These functions are related to the geometry of the element.
             * 
             * @param loc The local coordinates where the derivatives are to be evaluated.
             * 
             * @return Matrix The derivatives of the shape functions evaluated at the specified location.
             */
            virtual Matrix dNdxi(const Vector& loc)const=0;

            /**
             * @brief Get the derivatives of the shape functions with respect to global coordinates at a given location.
             * 
             * This method returns the derivatives of the shape functions with respect to the global coordinates, evaluated at the specified location. These functions are related to the geometry of the element.
             * 
             * @param element_nodes Vector of shared pointers to the nodes of the element.
             * @param loc The local coordinates where the derivatives are to be evaluated.
             * 
             * @return Matrix The derivatives of the shape functions evaluated at the specified location.
             */
            virtual Matrix dNdx(const VectorNodes& element_nodes,
                                const Vector& loc)const{

                Matrix dN_dxi = dNdxi(loc);
                Matrix Jacobian = J(element_nodes, loc);

                return Solver(Jacobian).solve(dN_dxi).transpose();
            }


    };

    /**
     * @brief Shared pointer type for IElementShape.
     */
    using IElementShape_ptr = std::shared_ptr<IElementShape>;


    /**
     * @class ElementShapeAdapter
     * 
     * @brief Adapter class to wrap specific element geometry implementations to the IElementShape interface.
     * 
     * @tparam ElShape The element geometry class to be adapted.
     */
    template<class ElShape>
    class ElementShapeAdapter: public IElementShape{

        public:

            ~ElementShapeAdapter()override = default;
            
            /**
             * @brief Get the shape type of the element.
             * 
             * @return ShapeType The shape type of the element.
             */
            ShapeType shape()const override{
                return ElShape::shape_type;
            }

            /**
             * @brief Get the number of nodes of the element.
             * 
             * @return int The number of nodes of the element.
             */
            int number_of_nodes()const override{
                return ElShape::number_of_nodes;
            }

            /**
             * @brief Get the number of edges of the element.
             * 
             * The number of edges may be different from the number of nodes.
             * For example, QUAD9 has 8 edges, but 9 nodes.
             * 
             * @return int The number of edges of the element.
             */
            int number_of_edges()const override{
                if constexpr(ElShape::number_of_dimensions>1){
                    return ElShape::number_of_edges;
                }else{
                    return 1;
                }
            }

            /**
             * @brief Get the number of faces of the element.
             * 
             * @return int The number of faces of the element.
             */
            int number_of_faces()const override{
                if constexpr(ElShape::number_of_dimensions>2){
                    return ElShape::number_of_faces;
                }else{
                    return 1;
                }
            }

            /**
             * @brief Get geometry of the element.
             * 
             * @return IGeometry_ptr The geometry of the element.
             */
            IGeometry_ptr geometry(const VectorNodes& element_nodes)const override{
                IGeometry_ptr geo;
                switch (number_of_dimensions())
                {
                case 1:
                    geo = std::make_shared<Line>(element_nodes[0],element_nodes[1]);
                    break;

                case 2:
                    geo = std::make_shared<IArea>(element_nodes);
                    break;

                case 3:
                    {
                        std::vector<Point> vertices;
                        vertices.reserve(element_nodes.size());
                        for(auto& pt : element_nodes){
                            vertices.emplace_back(*pt);
                        }
                        std::vector<IArea> faces;
                        std::vector<IArea_ptr> surfs = surfaces(element_nodes);
                        faces.reserve(surfs.size());
                        for(auto& surf : surfs){
                            faces.emplace_back(*surf);
                        }
                        geo = std::make_shared<IVolume>(vertices,faces);
                    }
                    break;
                
                default:
                    break;
                }
                return geo;
            }

             /**
             * @brief Get the number of dimensions of the element.
             * 
             * @return int The number of dimensions of the element.
             */
            int number_of_dimensions()const override{
                return ElShape::number_of_dimensions;
            }

            /**
             * @brief Get the integration geometry of the element.
             * 
             * @return IntegrationGeometry The integration geometry of the element.
             */
            IntegrationGeometry integration_domain()const override{
                return ElShape::integration_domain;
            }

            /**
             * @brief Get the integration order of the element.
             * 
             * @return int The integration order of the element.
             */
            int shape_order()const override{
                return ElShape::shape_order;
            }

            /**
             * @brief Compute the Jacobian matrix at a given location.
             * 
             * @param element_nodes Vector of shared pointers to the nodes of the element.
             * @param loc The local coordinates where the Jacobian is to be computed.
             * 
             * @return Matrix The Jacobian matrix at the specified location.
             */
            Matrix J(const VectorNodes& element_nodes,
                const Vector& loc=default_zero_vec(3))const override{
                return ElShape::J(element_nodes,loc);
            }

            /**
             * @brief Compute the determinant of the Jacobian at a given location.
             * 
             * @param element_nodes Vector of shared pointers to the nodes of the element.
             * @param loc The local coordinates where the determinant is to be computed.
             * 
             * @return double The determinant of the Jacobian at the specified location.
             */
            double detJ(const VectorNodes& element_nodes,
                const Vector& loc=default_zero_vec(3))const override{
                return ElShape::detJ(element_nodes,loc);
            }

            /**
             * @brief Get the edges of the element.
             * 
             * @param element_nodes Vector of shared pointers to the nodes of the element.
             * 
             * @return std::vector<Line_ptr> A vector of shared pointers to the edges of the element.
             */
            std::vector<Line_ptr> edges(const VectorNodes& element_nodes) const override{
                if constexpr(ElShape::number_of_dimensions>1){
                    std::vector<Line_ptr> lines;
                    lines.reserve(number_of_edges());
                    for(auto& nodes: ElShape::edge_nodes){
                        Point p1 = *element_nodes[nodes[0]];
                        Point p2 = *element_nodes[nodes[1]];
                        lines.emplace_back(std::make_shared<Line>(p1,p2));
                    }
                    return lines;
                }else{
                    std::vector<Line_ptr> lines;
                    return lines;
                }
            }

            /**
             * @brief Get the surfaces of the element.
             * 
             * @param element_nodes Vector of shared pointers to the nodes of the element.
             * 
             * @return std::vector<IArea_ptr> A vector of shared pointers to the surfaces of the element.
             */
            std::vector<IArea_ptr> surfaces(const VectorNodes& element_nodes)const override{
                if constexpr(ElShape::number_of_dimensions>2){
                    std::vector<IArea_ptr> surfaces;
                    surfaces.reserve(number_of_faces());
                    for(auto& nodes: ElShape::surface_nodes){
                        std::vector<Point> ps;
                        int num_nodes = nodes.size();
                        ps.reserve(num_nodes);
                        for(int i=0; i<num_nodes; i++){
                            ps.emplace_back(*element_nodes[nodes[i]]);
                        }
                        surfaces.emplace_back(std::make_shared<IArea>(ps));
                    }
                    return surfaces;
                }else{
                    std::vector<IArea_ptr> surfaces;
                    return surfaces;
                }
            }

            /**
             * @brief Get the shape functions at a given location.
             * 
             * This method returns the shape functions evaluated at the specified local coordinates. These functions are related to the geometry of the element.
             * 
             * @param loc The local coordinates where the shape functions are to be evaluated.
             * 
             * @return Vector The shape functions evaluated at the specified location.
             */
            Vector N(const Vector& loc)const override{
                return ElShape::N(loc);
            }

            /**
             * @brief Get the derivatives of the shape functions with respect to local coordinates at a given location.
             * 
             * This method returns the derivatives of the shape functions with respect to the local coordinates, evaluated at the specified location. These functions are related to the geometry of the element.
             * 
             * @param loc The local coordinates where the derivatives are to be evaluated.
             * 
             * @return Matrix The derivatives of the shape functions evaluated at the specified location.
             */
            Matrix dNdxi(const Vector& loc)const override{
                return ElShape::dNdxi(loc);
            }

            /**
             * @brief Get the derivatives of the shape functions with respect to global coordinates at a given location.
             * 
             * This method returns the derivatives of the shape functions with respect to the global coordinates, evaluated at the specified location. These functions are related to the geometry of the element.
             * 
             * @param element_nodes Vector of shared pointers to the nodes of the element.
             * @param loc The local coordinates where the derivatives are to be evaluated.
             * 
             * @return Matrix The derivatives of the shape functions evaluated at the specified location.
             */
            Matrix dNdx(const VectorNodes& element_nodes,
                        const Vector& loc) const override{
                return ElShape::dNdx(element_nodes,loc);
            }
    };
} // namespace finelc