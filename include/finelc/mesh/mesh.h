#pragma once

#include <finelc/enumerations.h>

#include <finelc/geometry/geometry.h>
#include <finelc/elements/element.h>
#include <finelc/mesh/element_finder.h>

#include <iostream>
#include <vector>
#include <memory>

namespace finelc{

    /**
     * @class Mesh
     * 
     * @brief Class representing a finite element mesh.
     */
    class Mesh{

        private:

            VectorNodes nodes; // Vector of shared pointers to the nodes in the mesh.
            VectorElements elements; // Vector of shared pointers to the elements in the mesh.
            ElementFinder_uptr finder;  // Unique pointer to the element finder.

        public:

            Mesh()=default;
            virtual ~Mesh()=default;

            /**
             * @brief Add a new element to the mesh.
             * 
             * @param new_el Shared pointer to the new element to be added.
             */
            void add_element(IElement_ptr new_el);

            /**
             * @brief Add multiple new elements to the mesh.
             * 
             * @param new_els Vector of shared pointers to the new elements to be added.
             */
            void add_elements(const VectorElements& new_els);

            /**
             * @brief Set the elements of the mesh.
             * 
             * @param els Vector of shared pointers to the elements to set.
             */
            void set_elements(const VectorElements& els);

            /**
             * @brief Add a new node to the mesh.
             * 
             * @param new_node Shared pointer to the new node to be added.
             */
            void add_node(Node_ptr new_node);

            /**
             * @brief Add multiple new nodes to the mesh.
             * 
             * @param new_nodes Vector of shared pointers to the new nodes to be added.
             */
            void add_nodes(const VectorNodes& new_nodes);

            /**
             * @brief Set the nodes of the mesh.
             * 
             * @param nds Vector of shared pointers to the nodes to set.
             */
            void set_nodes(const VectorNodes& nds);

            /**
             * @brief Set the element finder for the mesh.
             * 
             * The element finder is used to locate elements within the mesh based on given coordinates.
             * 
             * @param find Unique pointer to the element finder to set.
             */
            void set_finder(ElementFinder_uptr find);

            /**
             * @brief Get a specific element from the mesh.
             * 
             * @param el The index of the element to retrieve.
             * 
             * @return IElement_ptr Shared pointer to the requested element.
             */
            IElement_ptr get_element(int el)const;

            /**
             * @brief Get a specific node from the mesh.
             * 
             * @param nd The index of the node to retrieve.
             * 
             * @return Node_ptr Shared pointer to the requested node.
             */
            Node_ptr get_node(int nd)const;  

            /**
             * @brief Find a node from its coordinates
             * 
             * @param coord The coordinate of the node to retrieve
             * 
             * @return int Index of the node
             * 
             */
            int find_node(const Point& coord)const;

            /**
             * @brief Get all nodes in the mesh.
             * 
             * @return const VectorNodes& Reference to the vector of shared pointers to the nodes in the mesh.
             */
            const VectorNodes& get_nodes() const;

            /**
             * @brief Get all elements in the mesh.
             * 
             * @return const VectorElements& Reference to the vector of shared pointers to the elements in the mesh.
             */
            const VectorElements& get_elements() const;

            /**
             * @brief Compute the geometric centers of all elements in the mesh.
             * 
             * @return VectorNodes Vector of shared pointers to the points representing the geometric centers of the elements.
             */
            const VectorNodes element_center()const{
                VectorNodes centers;
                centers.reserve(elements.size());
                for(auto& el :elements){
                    centers.emplace_back(std::make_shared<Point>(el->geometric_center()));
                }
                return centers;
            }

            /**
             * @brief Get the number of nodes in the mesh.
             * 
             * @return size_t The number of nodes in the mesh.
             */
            size_t number_of_nodes() const;

            /**
             * @brief Get the number of elements in the mesh.
             * 
             * @return size_t The number of elements in the mesh.
             */
            size_t number_of_elements() const;

            /**
             * @brief Find the element containing a given location.
             * 
             * @param loc The location to search for.
             * 
             * @return int The index of the element containing the location.
             */
            int find_element(const Vector& loc) const;
    };

    /**
     * @brief Type alias for a shared pointer to a Mesh.
     */
    using Mesh_ptr = std::shared_ptr<Mesh>;

} // namespace finel