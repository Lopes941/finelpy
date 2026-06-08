#pragma once

#include <finelc/enumerations.h>
#include <finelc/matrix.h>

#include <finelc/element_physics/element_physics.h>

#include <array>
#include <vector>

namespace finelc{

    class Thermal1DConduction{

        public:

            inline static constexpr ModelType model = ModelType::THERMAL_CONDUCTION_1D;
            inline static constexpr int dof_per_node = 1;
            inline static constexpr bool linear = true;
            inline static constexpr std::array<DOFType,dof_per_node> dofs = {DOFType::T};

            static Matrix N(const Vector& N_geo, const Vector& loc);

            static Matrix dNdx(const Matrix& dNdx_geo, const Vector& loc);

            static Matrix B(const Matrix& dNdx,const Vector& loc);

            static Vector heat_flux(   const Vector& loc, 
                                    const Vector& ue,
                                    const Matrix& dNdx,
                                    const Matrix& D);

            static Vector compute( Tag_get_heat_flux, 
                            const Vector& loc, 
                            const Vector& ue,
                            const Matrix& dNdx,
                            const Matrix& D);
    };


    class Thermal2DConduction{

        public:

            inline static constexpr ModelType model = ModelType::THERMAL_CONDUCTION_2D;
            inline static constexpr int dof_per_node = 1;
            inline static constexpr bool linear = true;
            inline static constexpr std::array<DOFType,dof_per_node> dofs = {DOFType::T};

            static Matrix N(const Vector& N_geo, const Vector& loc);

            static Matrix dNdx(const Matrix& dNdx_geo, const Vector& loc);

            static Matrix B(const Matrix& dNdx,const Vector& loc);

            static Vector heat_flux(   const Vector& loc, 
                                    const Vector& ue,
                                    const Matrix& dNdx,
                                    const Matrix& D);

            static Vector compute( Tag_get_heat_flux, 
                            const Vector& loc, 
                            const Vector& ue,
                            const Matrix& dNdx,
                            const Matrix& D);

    };

    class Thermal3DConduction{

        public:

            inline static constexpr ModelType model = ModelType::THERMAL_CONDUCTION_3D;
            inline static constexpr int dof_per_node = 1;
            inline static constexpr bool linear = true;
            inline static constexpr std::array<DOFType,dof_per_node> dofs = {DOFType::T};

            static Matrix N(const Vector& N_geo, const Vector& loc);

            static Matrix dNdx(const Matrix& dNdx_geo, const Vector& loc);

            static Matrix B(const Matrix& dNdx,const Vector& loc);

            static Vector heat_flux(   const Vector& loc, 
                                    const Vector& ue,
                                    const Matrix& dNdx,
                                    const Matrix& D);

            static Vector compute( Tag_get_heat_flux, 
                            const Vector& loc, 
                            const Vector& ue,
                            const Matrix& dNdx,
                            const Matrix& D);

    };


} //namespace finelc