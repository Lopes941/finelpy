#pragma once

#include <finelc/enumerations.h>
#include <finelc/matrix.h>

#include <finelc/geometry/geometry.h>

#include <array>
#include <vector>

namespace finelc{

    class Tri3{

        public:

            inline static constexpr ShapeType shape_type = ShapeType::TRI3;
            inline static constexpr IntegrationGeometry integration_domain = IntegrationGeometry::TRIANGLE;
            inline static constexpr int number_of_nodes = 3;
            inline static constexpr int number_of_vertices = 3;
            inline static constexpr int shape_order = 1;
            inline static constexpr int number_of_dimensions = 2;

            inline static constexpr std::array<int[2],number_of_vertices> edge_nodes = {{
                {0,1},
                {1,2},
                {2,0}}
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
