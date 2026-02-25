#pragma once

#include <finelc/enumerations.h>
#include <finelc/matrix.h>

#include <finelc/geometry/geometry.h>

#include <vector>

namespace finelc{

    class Hex8{

        public:

            inline static constexpr ShapeType shape_type = ShapeType::HEX8;
            inline static constexpr IntegrationGeometry integration_domain = IntegrationGeometry::REGULAR;
            inline static constexpr int number_of_nodes = 8;
            inline static constexpr int number_of_edges = 12;
            inline static constexpr int number_of_faces = 6;
            inline static constexpr int shape_order = 1;
            inline static constexpr int number_of_dimensions = 3;

            inline static constexpr std::array<std::array<int, 4>,number_of_faces> surface_nodes = {{
                {0,1,2,3},
                {0,1,5,4},
                {1,2,6,5},
                {2,3,7,6},
                {3,0,4,7},
                {4,5,6,7}}
            };

            inline static constexpr std::array<int[2],number_of_edges> edge_nodes = {{
                {0,1},
                {1,2},
                {2,3},
                {3,0},
                {0,4},
                {1,5},
                {2,6},
                {3,7},
                {4,5},
                {5,6},
                {6,7},
                {7,4}}
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
