#pragma once

#include <finelc/matrix.h>

#include <finelc/elements/integration.h>

#include <memory>
#include <optional>
#include <functional>


namespace finelc{
    
    /**
     * @class ElementalMatrices
     * 
     * @brief Class to manage and compute elemental matrices such as stiffness and mass matrices.
     */
    class ElementalMatrices{

        private:

            /**
             * @brief Unique pointer to a matrix. 
            */
            using Matrix_uptr = std::unique_ptr<Matrix>;

            Matrix_uptr Ke=nullptr;
            Matrix_uptr Me=nullptr;

        public:

            ElementalMatrices()=default;
            ~ElementalMatrices()=default;

            /**
             * @brief Compute or retrieve the elemental stiffness matrix.
             * 
             * @tparam IPointsFn Type of the function to retrieve integration points and weights.
             * @tparam BFn Type of the function to compute the strain-displacement matrix B.
             * @tparam DFn Type of the function to compute the constitutive matrix D.
             * @tparam DetFn Type of the function to compute the determinant of the Jacobian.
             * 
             * @param ue The vector of nodal values (optional). Used for nonlinear physics/materials.
             * @param size The size of the stiffness matrix.
             * @param gaussfunc Function to retrieve the integration points and weights.
             * @param Bfunc Function to compute the strain-displacement matrix B.
             * @param Dfunc Function to compute the constitutive matrix D.
             * @param detJfunc Function to compute the determinant of the Jacobian.
             * 
             * @return const Matrix& The elemental stiffness matrix.
             */
            template<
                typename IPointsFn,
                typename BFn,
                typename DFn,
                typename DetFn
            >
            const Matrix& get_Ke(
                OptionalVector ue, 
                int size,
                IPointsFn&& gaussfunc,
                BFn&& Bfunc, 
                DFn&& Dfunc,
                DetFn&& detJfunc){
                    if(ue.has_value() || !Ke){
                        if(!Ke) Ke = std::make_unique<Matrix>(size,size);
                        Ke->setZero();

                        for(auto& pair: gaussfunc()){
                            Vector pi = pair.point.as_vector();
                            double wi = pair.weight;

                            Matrix Bmat = Bfunc(pi,ue);
                            *Ke += Bmat.transpose() * Dfunc(pi,ue) * Bmat * detJfunc(pi) * wi;
                        }
                    }
                    return *Ke;
                }

            /**
             * @brief Compute or retrieve the elemental mass matrix.
             * 
             * @tparam RhoCallerFn Type of the function to retrieve the density.
             * @tparam IPointsFn Type of the function to retrieve integration points and weights.
             * @tparam NFn Type of the function to compute the shape function matrix N.
             * @tparam DetFn Type of the function to compute the determinant of the Jacobian.
             * 
             * @param ue The vector of nodal values (optional). Used for nonlinear physics/materials.
             * @param size The size of the mass matrix.
             * @param rhofunc Function to retrieve the density.
             * @param gaussfunc Function to retrieve the integration points and weights.
             * @param Nfunc Function to compute the shape function matrix N.
             * @param detJfunc Function to compute the determinant of the Jacobian.
             * 
             * @return const Matrix& The elemental mass matrix.
             */
            template<
                typename RhoCallerFn,
                typename IPointsFn,
                typename NFn,
                typename DetFn
            >
            const Matrix& get_Me(
                OptionalVector ue, 
                int size,
                RhoCallerFn&& rhofunc,
                IPointsFn&& gaussfunc,
                NFn&& Nfunc,
                DetFn&& detJfunc){

                    if(ue.has_value() || !Me){
                        if(!Me) Me = std::make_unique<Matrix>(size,size);
                        Me->setZero();

                        for(auto& pair: gaussfunc()){
                            Vector pi = pair.point.as_vector();
                            double wi = pair.weight;

                            Matrix Nmat(Nfunc(pi,ue));
                            *Me += Nmat * Nmat.transpose() * detJfunc(pi) * wi;
                        }

                        *Me *= rhofunc();
                    }
                    return *Me;

            }

    };

    using ElementalMatrices_ptr = std::shared_ptr<ElementalMatrices>;

} // namespace finelc