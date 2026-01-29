#pragma once

namespace finelc{

    enum class ShapeType{
        PYTHON_SHAPE,
        LINE,
        TRI3,
        QUAD4,
        QUAD9,
        HEX8
    };

    enum class ModelType{
        PYTHON_PHYSICS,
        BAR_STRUCTURAL,
        BEAM_STRUCTURAL,
        PLANE_STRUCTURAL,
        SOLID_STRUCTURAL,
        THERMAL_CONDUCTION_1D,
        THERMAL_CONDUCTION_2D,
        THERMAL_CONDUCTION_3D
    };


    enum class ConstitutiveType{
        PYTHON_CONSTITUTIVE,
        BAR_LINEAR_ELASTIC,
        BEAM_LINEAR_ELASTIC,
        PLANE_STRESS,
        PLANE_STRAIN,
        SOLID_LINEAR_ELASTIC,
        THERMAL_CONDUCTION_1D,
        THERMAL_CONDUCTION_2D,
        THERMAL_CONDUCTION_3D
    };

    enum class IntegrationGeometry{
        REGULAR,
        TRIANGLE
    };


    enum class MaterialProperties{
        RHO,
        YOUNGS_MOD,
        YOUNGS_MOD_XX,
        YOUNGS_MOD_YY,
        YOUNGS_MOD_ZZ,
        POISSON,
        POISSON_XY,
        POISSON_XZ,
        POISSON_YZ,
        THERMAL_COND,
        THERMAL_COND_X,
        THERMAL_COND_Y,
        THERMAL_COND_Z,
        IZZ,
        A
    };

    enum class BoundaryConditionType{
        DIRICHLET
    };


    enum class AreaType{
        POLYGON,
        RECTANGLE,
    };


    enum class DOFType{
        UX,
        UY,
        UZ,
        THETAX,
        THETAY,
        THETAZ,
        T,
        P
    };

    enum class BCType{
        DIRICHLET,
        NEUMANN,
        EQUAL
    };

    enum class InterpolationScheme{
        NONE,
        SIMP,
    };

    enum class InterpolationParameters{
        P_EXPONENT,
        X_MIN,
        BETA,
        THRESHOLD,
    };

    enum class ResultData{
        UX,
        UY,
        UZ,
        THETAX,
        THETAY,
        THETAZ,
        ABS_U,
        EPSILON_XX,
        EPSILON_YY,
        EPSILON_ZZ,
        EPSILON_XY,
        EPSILON_XZ,
        EPSILON_YZ,
        SIGMA_XX,
        SIGMA_YY,
        SIGMA_ZZ,
        SIGMA_XY,
        SIGMA_XZ,
        SIGMA_YZ,
        SIGMA_VONMISES,
        NX,
        VY,
        MZ,
        
    };

} // namespace finelc
