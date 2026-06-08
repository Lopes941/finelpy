#pragma once

#include <finelc/enumerations.h>
#include <finelc/matrix.h>

#include <finelc/material/material.h>

#include <array>
#include <vector>

namespace finelc{

    class Thermal1D{

        public:

            inline static constexpr ConstitutiveType model_name = ConstitutiveType::THERMAL_CONDUCTION_1D;
            inline static constexpr bool linear = true;

            static Matrix D(Material_ptr material,
                            const Vector&, 
                            const Vector&){


                DenseMatrix D(1,1);

                double k = material->get_property(MaterialProperties::THERMAL_COND);

                D << k;

                return D;

            }

    };

    class Thermal2D{

        public:

            inline static constexpr ConstitutiveType model_name = ConstitutiveType::THERMAL_CONDUCTION_2D;
            inline static constexpr bool linear = true;

            static Matrix D(Material_ptr material,
                            const Vector&, 
                            const Vector&){

                double k = material->get_property(MaterialProperties::THERMAL_COND);

                DenseMatrix D(2,2);

                D <<
                    k, 0,
                    0, k;

                return D;
            }

    };

    class Thermal3D{

        public:

            inline static constexpr ConstitutiveType model_name = ConstitutiveType::THERMAL_CONDUCTION_3D;
            inline static constexpr bool linear = true;

            static Matrix D(Material_ptr material,
                            const Vector&, 
                            const Vector&){

                double k = material->get_property(MaterialProperties::THERMAL_COND);

                DenseMatrix D(3,3);

                D <<
                    k, 0, 0,
                    0, k, 0,
                    0, 0, k;
                
                return D;
            }

    };

} // namespace finelc