#pragma once

#include <finelc/enumerations.h>
#include <finelc/matrix.h>

#include <finelc/material/material.h>

#include <unordered_map>
#include <mutex>
#include <memory>
#include <string>


namespace finelc{

    /**
     * @class IConstitutiveModel
     * 
     * @brief Interface for constitutive material models.
     */
    class IConstitutiveModel{

            protected:
                Material_ptr material; // Pointer to the material associated with the constitutive model

            public:

                virtual ~IConstitutiveModel() = default;

                /**
                 * @brief Get the value of a material property.
                 * 
                 * @param prop The material property to retrieve.
                 * 
                 * @return double The value of the requested material property.
                 */
                double get_property(const MaterialProperties& prop){
                    return material->get_property(prop);
                }

                /**
                 * @brief Check if the material is linear.
                 * 
                 * @return true if the material is linear, false otherwise.
                 */
                virtual bool is_linear()const=0;

                /**
                 * @brief Get the constitutive model type.
                 * 
                 * @return ConstitutiveType The constitutive model type.
                 */
                virtual ConstitutiveType contitutive_model()const=0;

                /**
                 * @brief Get the constitutive matrix at a given location.
                 * 
                 * @param loc The local coordinates where the constitutive matrix is to be evaluated.
                 * @param ue The vector of nodal values.
                 * 
                 * @return Matrix The constitutive matrix at the specified location.
                 */
                virtual Matrix D(const Vector& loc, const Vector& ue)const=0;

        };

        /**
         * @brief Shared pointer type for IConstitutiveModel objects.
         */
        using IConstitutiveModel_ptr = std::shared_ptr<IConstitutiveModel>;

        /**
         * @class ConstitutiveModelAdapter
         * 
         * @brief Adapter class to wrap specific constitutive model implementations.
         * 
         * @tparam ConstModel The specific constitutive model class to adapt.
         */
        template<class ConstModel>
        class ConstitutiveModelAdapter: public IConstitutiveModel{

            public:

                /** 
                 * @brief Constructor that initializes the adapter with a material pointer.
                 * 
                 * @param material_ Shared pointer to the material associated with the constitutive model.
                */
                ConstitutiveModelAdapter(Material_ptr material_){
                    material = material_;
                }
                
                ~ConstitutiveModelAdapter()override = default;


                /**
                 * @brief Get the constitutive model type.
                 * 
                 * @return ConstitutiveType The constitutive model type.
                 */
                ConstitutiveType contitutive_model()const override{
                    return ConstModel::model_name;
                }


                /**
                 * @brief Check if the material is linear.
                 * 
                 * @return true if the material is linear, false otherwise.
                 */
                bool is_linear()const override{
                    return ConstModel::linear;
                }


                /**
                 * @brief Get the constitutive matrix at a given location.
                 * 
                 * @param loc The local coordinates where the constitutive matrix is to be evaluated.
                 * @param ue The vector of nodal values.
                 * 
                 * @return Matrix The constitutive matrix at the specified location.
                 */
                Matrix D(const Vector& loc, 
                        const Vector& ue)const override{
                    return ConstModel::D(material, loc, ue);
                }
        };

} // namespace finelc