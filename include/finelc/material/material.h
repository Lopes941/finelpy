#pragma once

#include <finelc/enumerations.h>
#include <finelc/matrix.h>

#include <unordered_map>
#include <mutex>
#include <memory>
#include <string>

namespace finelc{

    /**
     * @class Material
     * 
     * @brief Class representing a material with various properties.
     */
    class Material{

        private:

            std::string material_id; // Identifier for the material
            std::unordered_map<MaterialProperties, double> properties_map; // List of given material properties.

        public:

            Material()=default;
            ~Material()=default;

            /**
             * @brief Set the material identifier.
             * 
             * @param identifier The identifier to set.
             * @param value The value of the property to set.
             * 
             * @return Material& Reference to the current Material object.
             */
            Material& add_property(const MaterialProperties& identifier, const double value);


            /**
             * @brief Get the value of a material property.
             * 
             * @param identifier The material property identifier.
             * 
             * @return double The value of the requested material property.
             */
            double get_property(const MaterialProperties& identifier) const;
            
    };

    /**
     * @brief Shared pointer type for Material objects.
     */
    using Material_ptr = std::shared_ptr<Material>;


    /**
     * @class MaterialCatalogue
     * 
     * @brief Singleton class for managing a catalogue of materials.
     */
    class MaterialCatalogue{

        private:

            std::unordered_map<std::string, Material_ptr> materials;

            static std::shared_ptr<MaterialCatalogue> catalogue;

            static std::mutex mtx;

            MaterialCatalogue() {}

            static inline bool check_existence() {return catalogue != nullptr;}
            static inline void create_instance() {
                std::lock_guard<std::mutex> lock(mtx);
                if(!check_existence()){
                    catalogue = std::shared_ptr<MaterialCatalogue>(new MaterialCatalogue());
            }}

        public:

            MaterialCatalogue(const MaterialCatalogue& obj) = delete;
            

            inline static std::shared_ptr<MaterialCatalogue> get_catalogue(){
                if(!check_existence()){
                    create_instance();
                }
                return catalogue;
            }

            Material_ptr get_material(const std::string& material_id);
            Material_ptr create_material(const std::string& material_id);

    };

 
} // namespace finelc