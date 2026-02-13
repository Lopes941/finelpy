#pragma once

#include <finelc/enumerations.h>
#include <finelc/matrix.h>

#include <finelc/geometry/geometry.h>

#include <vector>

namespace finelc{

    class Quad4{

        public:

            inline static constexpr ShapeType shape_type = ShapeType::QUAD4;
            inline static constexpr IntegrationGeometry integration_domain = IntegrationGeometry::REGULAR;
            inline static constexpr int number_of_nodes = 4;
            inline static constexpr int number_of_vertices = 4;
            inline static constexpr int shape_order = 1;
            inline static constexpr int number_of_dimensions = 2;

            inline static constexpr std::array<int[2],number_of_vertices> edge_nodes = {{
                {0,1},
                {1,2},
                {2,3},
                {3,0}}
            };
            
            static Vector N(const Vector& loc);
            static Matrix dNdxi(const Vector& loc);

            static Matrix dNdx( const VectorNodes& element_nodes,
                                const Vector& loc);

            static Matrix J(const VectorNodes& element_nodes,
                    const Vector& loc);
                    
            static double detJ(const VectorNodes& element_nodes,
                    const Vector& loc);

    };


    class Quad9{

        public:

            inline static constexpr ShapeType shape_type = ShapeType::QUAD9;
            inline static constexpr IntegrationGeometry integration_domain = IntegrationGeometry::REGULAR;
            inline static constexpr int number_of_nodes = 9;
            inline static constexpr int number_of_vertices = 8;
            inline static constexpr int shape_order = 2;
            inline static constexpr int number_of_dimensions = 2;

            inline static constexpr std::array<int[2],number_of_vertices> edge_nodes = {{
                {0,4},
                {4,1},
                {1,5},
                {5,2},
                {2,6},
                {6,3},
                {3,7},
                {7,0}}
            };


            static Vector N(const Vector& loc);
            static Matrix dNdxi(const Vector& loc);

            static Matrix dNdx( const VectorNodes& element_nodes,
                                const Vector& loc);

            static Matrix J(const VectorNodes& element_nodes,
                    const Vector& loc);
                    
            static double detJ(const VectorNodes& element_nodes,
                    const Vector& loc);


    };


} // namespace finelc
