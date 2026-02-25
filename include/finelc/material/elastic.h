#pragma once

#include <finelc/enumerations.h>
#include <finelc/matrix.h>

#include <finelc/material/material.h>

#include <array>
#include <vector>

namespace finelc{

    class BarLinearElastic{

        public:

            inline static constexpr ConstitutiveType model_name = ConstitutiveType::BAR_LINEAR_ELASTIC;
            inline static constexpr bool linear = true;

            static Matrix D(Material_ptr material,
                            const Vector&, 
                            const Vector&){


            DenseMatrix D(1,1);

            double E = material->get_property(MaterialProperties::YOUNGS_MOD);
            double A = material->get_property(MaterialProperties::A);

            D << E*A;

            return D;

            }

    };

    class PlaneStress{

        public:

            inline static constexpr ConstitutiveType model_name = ConstitutiveType::PLANE_STRESS;
            inline static constexpr bool linear = true;

            static Matrix D(Material_ptr material,
                            const Vector&, 
                            const Vector&){

                double E = material->get_property(MaterialProperties::YOUNGS_MOD);
                double nu = material->get_property(MaterialProperties::POISSON);

                DenseMatrix D(3,3);

                D <<
                    1, nu,          0,
                    nu,  1,          0,
                    0,  0, (1.-nu)/2.;
                D*= E/(1-nu*nu);

                return D;
            }

    };

    class PlaneStrain{

        public:

            inline static constexpr ConstitutiveType model_name = ConstitutiveType::PLANE_STRAIN;
            inline static constexpr bool linear = true;

            static Matrix D(Material_ptr material,
                            const Vector&, 
                            const Vector&){

                double E = material->get_property(MaterialProperties::YOUNGS_MOD);
                double nu = material->get_property(MaterialProperties::POISSON);

                DenseMatrix D(3,3);

                D <<
                    1.-nu,     nu,           0,
                    nu,  1.-nu,           0,
                        0,      0, (1-2*nu)/2.;
                D*= E/((1+nu)*(1-2*nu));
                
                return D;
            }

    };

    class LinearElastic{

        public:

            inline static constexpr ConstitutiveType model_name = ConstitutiveType::SOLID_LINEAR_ELASTIC;
            inline static constexpr bool linear = true;

            static Matrix D(Material_ptr material,
                            const Vector&, 
                            const Vector&){

                double E = material->get_property(MaterialProperties::YOUNGS_MOD);
                double nu = material->get_property(MaterialProperties::POISSON);

                DenseMatrix D(6,6);

                D <<
                    1.-nu, nu, nu, 0, 0, 0,
                    nu, 1.-nu, nu, 0, 0, 0,
                    nu, nu, 1.-nu, 0, 0, 0,
                    0, 0, 0, (1.-2.*nu)/2., 0, 0,
                    0, 0, 0, 0, (1.-2.*nu)/2., 0,
                    0, 0, 0, 0, 0, (1.-2.*nu)/2.;
                D*= E/((1.+nu)*(1.-2.*nu));
                
                return D;
            }

    };


} // namespace finelc