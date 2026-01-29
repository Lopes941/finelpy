#pragma once

#include <finelc/matrix.h>
#include <finelc/enumerations.h>


#include <unordered_map>
#include <vector>
#include <stdexcept>
#include <string>
#include <numeric>


namespace finelc{

    class InterpolationProperties{

        private:
            std::unordered_map<InterpolationParameters, double> properties_map;

        public:

            InterpolationProperties()=default;
            ~InterpolationProperties()=default;

            InterpolationProperties& add_property(const InterpolationParameters& identifier, const double value){
                properties_map[identifier] = value;
                return *this;
            }

            double get_property(const InterpolationParameters& identifier) const{
                auto it = properties_map.find(identifier);
                if (it == properties_map.end()) {
                    throw std::out_of_range("Property '" + std::to_string(static_cast<int>(identifier)) + "' not found in Interpolation Scheme.");
                }
                return it->second;
            }

    };

    class IInterpolationScheme{

        protected:
            InterpolationProperties properties;

        public:

            virtual ~IInterpolationScheme()=default;
            
            IInterpolationScheme& update_property(
                                const InterpolationParameters& identifier, 
                                const double value){
                properties.add_property(identifier,value);
                return *this;
            }

            virtual InterpolationScheme name()=0;
            virtual Vector apply(const Vector& rho)=0;
            virtual Vector derivative(const Vector& rho)=0;

    };

    

    template<class Scheme>
    class InterpolationSchemeAdapter: public IInterpolationScheme{

        public:
            InterpolationSchemeAdapter()=default;
            ~InterpolationSchemeAdapter() override=default;

            InterpolationScheme name() override{
                return Scheme::name;
            }

            Vector apply(const Vector& rho) override{
                return Scheme::apply(rho,properties);
            }

            Vector derivative(const Vector& rho) override{
                return Scheme::derivative(rho,properties);
            }

    };

    class NoInterpolation{

        public:
            inline static constexpr InterpolationScheme name = InterpolationScheme::NONE;

            static Vector apply(
                            const Vector& rho, 
                            const InterpolationProperties& ){
                return rho;
            }

            static Vector derivative(
                            const Vector& rho, 
                            const InterpolationProperties& ){
                return rho;
            }
        
    };


    class SIMPInterpolationScheme{

        public:
            inline static constexpr InterpolationScheme name = InterpolationScheme::SIMP;

            static Vector apply(
                            const Vector& rho, 
                            const InterpolationProperties& prop){

                double p    = prop.get_property(InterpolationParameters::P_EXPONENT);
                Vector interp(rho.size());

                for(int i=0; i<rho.size(); i++){
                    interp(i) = std::pow(rho(i),p);
                }

                return interp;
            }

            static Vector derivative(
                            const Vector& rho, 
                            const InterpolationProperties& prop){

                double p    = prop.get_property(InterpolationParameters::P_EXPONENT);
                double p1   = p-1;
                Vector interp(rho.size());

                for(int i=0; i<rho.size(); i++){
                    interp(i) = p*std::pow(rho(i),p1);
                }

                return interp;
            }

    };


} // namespace finelc