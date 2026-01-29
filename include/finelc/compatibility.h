#pragma once

#include <finelc/enumerations.h>

#include <unordered_map>
#include <unordered_set>

namespace finelc{

    
    inline std::unordered_map<ShapeType, std::unordered_set<ModelType>> valid_shape_model = {
        { 
            ShapeType::LINE,{ 
                ModelType::BAR_STRUCTURAL, 
                ModelType::BEAM_STRUCTURAL,
                ModelType::THERMAL_CONDUCTION_1D }},
        { 
            ShapeType::QUAD4,{ 
                ModelType::PLANE_STRUCTURAL,
                ModelType::THERMAL_CONDUCTION_2D }},
        { 
            ShapeType::QUAD9,{ 
                ModelType::PLANE_STRUCTURAL,
                ModelType::THERMAL_CONDUCTION_2D }},
        { 
            ShapeType::TRI3,{ 
                ModelType::PLANE_STRUCTURAL,
                ModelType::THERMAL_CONDUCTION_2D }},

        { 
            ShapeType::HEX8,{ 
                ModelType::SOLID_STRUCTURAL,
                ModelType::THERMAL_CONDUCTION_3D }}
    };


    inline std::unordered_map<ModelType, std::unordered_set<ConstitutiveType>> valid_model_constitutive = {
        { 
            ModelType::BAR_STRUCTURAL,{  
                ConstitutiveType::BAR_LINEAR_ELASTIC }},
        { 
            ModelType::BEAM_STRUCTURAL,{  
                ConstitutiveType::BEAM_LINEAR_ELASTIC }},
        { 
            ModelType::PLANE_STRUCTURAL,{
                ConstitutiveType::PLANE_STRESS,
                ConstitutiveType::PLANE_STRAIN,  }},
        { 
            ModelType::SOLID_STRUCTURAL,{
                ConstitutiveType::SOLID_LINEAR_ELASTIC  }},
        { 
            ModelType::THERMAL_CONDUCTION_1D,{ 
                ConstitutiveType::THERMAL_CONDUCTION_1D }},
        { 
            ModelType::THERMAL_CONDUCTION_2D,{ 
                ConstitutiveType::THERMAL_CONDUCTION_2D }},
        { 
            ModelType::THERMAL_CONDUCTION_3D,{ 
                ConstitutiveType::THERMAL_CONDUCTION_3D }},
    };

    void element_compatibility( ShapeType shape_type,
                                ModelType physics_type,
                                ConstitutiveType const_type);

} // namespace finelc
