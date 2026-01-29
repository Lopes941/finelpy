#pragma once

#include <finelc/enumerations.h>
#include <finelc/matrix.h>

#include <finelc/element_physics/element_physics.h>

#include <array>
#include <vector>

namespace finelc{

    class PlaneStructural{

        public:

            inline static constexpr ModelType model = ModelType::PLANE_STRUCTURAL;
            inline static constexpr int dof_per_node = 2;
            inline static constexpr bool linear = true;
            inline static constexpr std::array<DOFType,dof_per_node> dofs = {DOFType::UX, DOFType::UY};

            static Matrix N(const Vector& N_geo, const Vector& loc);

            static Matrix dNdx(const Matrix& dNdx_geo, const Vector& loc);

            static Matrix B(const Matrix& dNdx,const Vector& loc);

            static Vector strain(   const Vector& loc, 
                                    const Vector& ue,
                                    const Matrix& dNdx);

            static Vector stress(   const Vector& loc, 
                                    const Vector& ue,
                                    const Matrix& dNdx,
                                    const Matrix& D);

            static Vector compute( Tag_get_strain, 
                            const Vector& loc, 
                            const Vector& ue,
                            const Matrix& dNdx,
                            const Matrix&);

            static Vector compute( Tag_get_stress, 
                            const Vector& loc, 
                            const Vector& ue,
                            const Matrix& dNdx,
                            const Matrix& D);

    };

} //namespace finelc