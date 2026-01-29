#pragma once

#include <finelc/enumerations.h>

#include <finelc/geometry/geometry.h>

#include <finelc/elements/element.h>

#include <finelc/mesh/mesh.h>
#include <finelc/mesh/element_finder.h>

#include <iostream>
#include <vector>
#include <memory>

namespace finelc{

    /**
     * @class MeshBuilder
     * 
     * @brief Abstract base class for building finite element meshes.
     */
    class MeshBuilder{

        protected:

            Mesh_ptr mesh; // The mesh being built.
            IElement_ptr element; // The element type used in the mesh.
            IGeometry_ptr domain; // The geometric domain of the mesh.

            MeshBuilder(IElement_ptr el): element(el), mesh(std::make_shared<Mesh>()) {}


        public:

            virtual ~MeshBuilder()=default;

            /**
             * @brief Build and return the constructed mesh.
             * 
             * @return Mesh_ptr A shared pointer to the constructed mesh.
             */
            virtual Mesh_ptr build()=0;
            
    };

    /**
     * @class MeshBuilderRectangle
     * 
     * @brief Class for building rectangular finite element meshes.
     */
    class MeshBuilderRectangle: public MeshBuilder{

        private:

            double Lx=0, Ly=0; // Dimensions of the rectangle
            double lx=0, ly=0; // Element sizes in x and y directions
            int nx=0, ny=0; // Number of elements in x and y directions

            /**
             * @brief Set the size of the rectangle based on the grid and element sizes.
             */
            void set_size_from_grid();

            /**
             * @brief Set the grid based on the rectangle size and element sizes.
             */
            void create_square();

            /**
             * @brief Populate the mesh with QUAD4 elements.
             */
            void populate_quad4();

            /**
             * @brief Populate the mesh with QUAD9 elements.
             */
            void populate_quad9();

            /**
             * @brief Populate the mesh with TRI3 elements.
             */
            void populate_tri3();

        public:

            MeshBuilderRectangle(
                std::shared_ptr<IArea> domain_, 
                IElement_ptr el);

            ~MeshBuilderRectangle()=default;

            /**
             * @brief Set the size of the rectangle.
             * 
             * @param lx_ Element size in the x direction.
             * @param ly_ Element size in the y direction.
             */
            void set_element_size(double lx_, double ly_);

            /**
             * @brief Set the grid of the rectangle.
             * 
             * @param nx_ Number of elements in the x direction.
             * @param ny_ Number of elements in the y direction.
             */
            void set_grid(int nx_, int ny_);

            /**
             * @brief Build and return the constructed mesh.
             * 
             * @return Mesh_ptr A shared pointer to the constructed mesh.
             */
            Mesh_ptr build() final;


    };

    /**
     * @class MeshBuilderLine
     * 
     * @brief Class for building line finite element meshes.
     */
    class MeshBuilderLine: public MeshBuilder{

        private:

            double L=0; // Dimensions of the line
            double l=0; // Element sizes
            int num_elements=0; // Number of elements

            /**
             * @brief Set the grid based on the line size and element sizes.
             */
            void create_line();

        public:

            MeshBuilderLine(
                std::shared_ptr<Line> domain_, 
                IElement_ptr el);
                
            ~MeshBuilderLine()=default;

            /**
             * @brief Set the grid from the number of elements.
             * 
             * @param n Number of elements.
             */
            void create_from_element_num(int n);

            /**
             * @brief Set the grid from the element size.
             * 
             * @param dL Element size.
             */
            void create_from_element_size(double dL);

            /**
             * @brief Build and return the constructed mesh.
             * 
             * @return Mesh_ptr A shared pointer to the constructed mesh.
             */
            Mesh_ptr build() final;


    };

    /**
     * @class MeshBuilderFrame
     * 
     * @brief Class for building finite element meshes composed of frames
     */
    class MeshBuilderFrame: public MeshBuilder{

        private:

            VectorNodes nodes; // Coordinate of nodes
            std::vector<std::vector<int>> inci; // Incidence of elements

            /**
             * @brief Set the frame based on the coordinate of nodes and element indices.
             */
            void create_frame();

        public:

            MeshBuilderFrame(
                IElement_ptr el_,
                VectorNodes nodes_,
                std::vector<std::vector<int>> inci_);
                
            ~MeshBuilderFrame()=default;

            /**
             * @brief Build and return the constructed mesh.
             * 
             * @return Mesh_ptr A shared pointer to the constructed mesh.
             */
            Mesh_ptr build() final;


    };
} // namespace finel