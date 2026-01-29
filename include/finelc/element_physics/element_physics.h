#pragma once


#include <finelc/matrix.h>
#include <finelc/enumerations.h>

#include <finelc/geometry/geometry.h>

#include <vector>
#include <memory>
#include <type_traits>
#include <utility> 
#include <stdexcept>

namespace finelc{


    class IElementPhysics{

        public:

            virtual ~IElementPhysics() = default;
            
            /**
             * @brief Get the physics model type of the element.
             * 
             * @return ModelType The physics model type of the element.
             */
            virtual ModelType get_model()const=0;

            /**
             * @brief Get the number of degrees of freedom per node.
             * 
             * @return int The number of degrees of freedom per node.
             */
            virtual int dof_per_node()const=0;

            /**
             * @brief Check if the physics is linear.
             * 
             * @return true if the physics is linear, false otherwise.
             */
            virtual bool is_linear()const=0;

            /**
             * @brief Get the degrees of freedom of the element.
             * 
             * @return std::vector<DOFType> A vector of degrees of freedom types for the element.
             */
            virtual std::vector<DOFType> dofs() const=0;

            /**
             * @brief Get the interpolation functions for the physics at a given location.
             * 
             * This method returns the interpolation functions evaluated at the specified local coordinates. These functions are related to the output vector or scalar field of the problem.
             * 
             * @param N_geo The shape functions from the geometry of the element.
             * @param loc The local coordinates where the interpolation functions are to be evaluated.
             * 
             * @return Matrix The interpolation functions evaluated at the specified location.
             */
            virtual Matrix N(const Vector& N_geo, const Vector& loc) const=0;

            /**
             * @brief Get the derivatives of the interpolation functions with respect to global coordinates at a given location.
             * 
             * This method returns the derivatives of the interpolation functions with respect to the global coordinates, evaluated at the specified location. These functions are related to the output vector or scalar field of the problem.
             * 
             * @param dNdx_geo The derivatives of the shape functions from the geometry of the element with respect to global coordinates.
             * @param loc The local coordinates where the interpolation functions are to be evaluated.
             * 
             * @return Matrix The derivatives of the interpolation functions evaluated at the specified location.
             */
            virtual Matrix dNdx(const Matrix& dNdx_geo, const Vector& loc) const=0;

            /**
             * @brief Get the strain-displacement matrix at a given location.
             * 
             * @param dNdx The derivatives of the interpolation functions with respect to global coordinates.
             * @param loc The local coordinates where the interpolation functions are to be evaluated.
             * 
             * @return Matrix The strain-displacement matrix at the specified location.
             */
            virtual Matrix B(const Matrix& dNdx, const Vector& loc)const = 0;


            /*********************RESULT ACCESS METHODS**************************/

            // ELASTICITY RESULTS

            /**
             * @brief Check if the element supports strain computation.
             * 
             * @return false by default. Must be overridden in derived classes.
             */
            virtual bool supports_strain()const{return false;}

            /**
             * @brief Get the strain at a given location.
             * 
             * @param loc The local coordinates where the strain is to be computed.
             * @param ue The vector of nodal values.
             * @param dNdx The derivatives of the interpolation functions with respect to global coordinates.
             * @param D The constitutive matrix.
             * 
             * @return Vector The strain at the specified location.
             */
            virtual Vector get_strain(  const Vector& loc, 
                                        const Vector& ue, 
                                        const Matrix& dNdx,
                                        const Matrix& D)const;

            /**
             * @brief Check if the element supports stress computation.
             * 
             * @return false by default. Must be overridden in derived classes.
             */
            virtual bool supports_stress()const{return false;}

            /**
             * @brief Get the stress at a given location.
             * 
             * @param loc The local coordinates where the stress is to be computed.
             * @param ue The vector of nodal values.
             * @param dNdx The derivatives of the interpolation functions with respect to global coordinates.
             * @param D The constitutive matrix.
             * 
             * @return Vector The stress at the specified location.
             */
            virtual Vector get_stress(  const Vector& loc, 
                                        const Vector& ue, 
                                        const Matrix& dNdx,
                                        const Matrix& D)const;

            /**
             * @brief Check if the element supports axial force computation.
             * 
             * @return false by default. Must be overridden in derived classes.
             */
            virtual bool supports_NX()const{return false;}

            /**
             * @brief Get the axial force at a given location.
             * 
             * @param loc The local coordinates where the axial force is to be computed.
             * @param ue The vector of nodal values.
             * @param dNdx The derivatives of the interpolation functions with respect to global coordinates.
             * @param D The constitutive matrix.
             * 
             * @return Vector The axial force at the specified location.
             */
            virtual Vector get_NX(  const Vector& loc, 
                                        const Vector& ue, 
                                        const Matrix& dNdx,
                                        const Matrix& D)const;

            /**
             * @brief Check if the element supports shear force computation.
             * 
             * @return false by default. Must be overridden in derived classes.
             */
            virtual bool supports_VY()const{return false;}

            /**
             * @brief Get the shear force at a given location.
             * 
             * @param loc The local coordinates where the shear force is to be computed.
             * @param ue The vector of nodal values.
             * @param dNdx The derivatives of the interpolation functions with respect to global coordinates.
             * @param D The constitutive matrix.
             * 
             * @return Vector The shear force at the specified location.
             */
            virtual Vector get_VY(  const Vector& loc, 
                                        const Vector& ue, 
                                        const Matrix& dNdx,
                                        const Matrix& D)const;

            /**
             * @brief Check if the element supports bending moment computation.
             * 
             * @return false by default. Must be overridden in derived classes.
             */
            virtual bool supports_MZ()const{return false;}

            /**
             * @brief Get the bending moment at a given location.
             * 
             * @param loc The local coordinates where the bending moment is to be computed.
             * @param ue The vector of nodal values.
             * @param dNdx The derivatives of the interpolation functions with respect to global coordinates.
             * @param D The constitutive matrix.
             * 
             * @return Vector The bending moment at the specified location.
             */
            virtual Vector get_MZ(  const Vector& loc, 
                                        const Vector& ue, 
                                        const Matrix& dNdx,
                                        const Matrix& D)const;
            
            

            
    };

    /**
     * @brief Shared pointer type for IElementPhysics.
     */
    using IElementPhysics_ptr = std::shared_ptr<IElementPhysics>;


    /**
     * @brief Helper function to throw an error message for unsupported methods.
     * 
     * @param name The name of the unsupported method.
     * 
     * @throws std::runtime_error indicating that the method is not supported.
     */
    inline void error_message(std::string name){
        throw std::runtime_error(name + " not supported by this physics");
    }

    struct Tag_get_strain {}; // tag to identify strain computation
    struct Tag_get_stress {}; // tag to identify stress computation
    struct Tag_Mz {}; // tag to identify Mz computation
    struct Tag_Vy {}; // tag to identify Vy computation
    struct Tag_Nx {}; // tag to identify Nx computation

    /**
     * @brief Trait to check if a specific result computation is supported by the element physics.
     * 
     * @tparam ElPhysics The element physics class to be checked.
     * @tparam Tag The tag identifying the specific result computation.
     * @tparam void SFINAE helper.
     * 
     * @return true_type if the computation is supported, false_type otherwise.
     */
    template <class ElPhysics, class Tag, class = void>
    struct SupportsResult : std::false_type {};

    /**
     * @brief Specialization of SupportsResult for supported computations.
     * 
     * @tparam ElPhysics The element physics class to be checked.
     * @tparam Tag The tag identifying the specific result computation.
     * @tparam void SFINAE helper.
     * 
     * @return true_type indicating that the computation is supported.
     */
    template <class ElPhysics, class Tag>
    struct SupportsResult<
        ElPhysics,
        Tag,
        std::void_t<
            decltype(ElPhysics::compute(
                std::declval<Tag>(),
                std::declval<Vector>(),
                std::declval<Vector>(),
                std::declval<Matrix>(),
                std::declval<Matrix>()
            ))
        >
    > : std::true_type {};

    /**
     * @class ElementPhysicsAdapter
     * 
     * @brief Adapter class to wrap specific element physics implementations to the IElementPhysics interface.
     * 
     * @tparam ElPhysics The element physics class to be adapted.
     */
    template <class ElPhysics>
    class ElementPhysicsAdapter: public IElementPhysics{

        public:

            ~ElementPhysicsAdapter() override= default;

            /**
             * @brief Get the physics model type of the element.
             * 
             * @return ModelType The physics model type of the element.
             */
            ModelType get_model()const override{
                return ElPhysics::model;
            }

            /**
             * @brief Get the number of degrees of freedom per node.
             * 
             * @return int The number of degrees of freedom per node.
             */
            int dof_per_node()const override{
                return ElPhysics::dof_per_node;
            }

            /**
             * @brief Check if the physics is linear.
             * 
             * @return true if the physics is linear, false otherwise.
             */
            bool is_linear()const override{
                return ElPhysics::linear;
            }

            /**
             * @brief Get the degrees of freedom of the element.
             * 
             * @return std::vector<DOFType> A vector of degrees of freedom types for the element.
             */
            std::vector<DOFType> dofs() const override{
                std::vector<DOFType> eldofs;
                eldofs.reserve(dof_per_node());
                for(auto& d: ElPhysics::dofs){
                    eldofs.emplace_back(d);
                }
                return eldofs;
            }

            /**
             * @brief Get the interpolation functions for the physics at a given location.
             * 
             * This method returns the interpolation functions evaluated at the specified local coordinates. These functions are related to the output vector or scalar field of the problem.
             * 
             * @param N_geo The shape functions from the geometry of the element.
             * @param loc The local coordinates where the interpolation functions are to be evaluated.
             * 
             * @return Matrix The interpolation functions evaluated at the specified location.
             */
            Matrix N(const Vector& N_geo, const Vector& loc) const override{
                return ElPhysics::N(N_geo, loc);
            }

            /**
             * @brief Get the derivatives of the interpolation functions with respect to global coordinates at a given location.
             * 
             * This method returns the derivatives of the interpolation functions with respect to the global coordinates, evaluated at the specified location. These functions are related to the output vector or scalar field of the problem.
             * 
             * @param dNdx_geo The derivatives of the shape functions from the geometry of the element with respect to global coordinates.
             * @param loc The local coordinates where the interpolation functions are to be evaluated.
             * 
             * @return Matrix The derivatives of the interpolation functions evaluated at the specified location.
             */
            Matrix dNdx(const Matrix& dNdx_geo, const Vector& loc) const override{
                return ElPhysics::dNdx(dNdx_geo, loc);
            }

            /**
             * @brief Get the strain-displacement matrix at a given location.
             * 
             * @param dNdx The derivatives of the interpolation functions with respect to global coordinates.
             * @param loc The local coordinates where the interpolation functions are to be evaluated.
             * 
             * @return Matrix The strain-displacement matrix at the specified location.
             */
            Matrix B(const Matrix& dNdx, const Vector& loc)const override{
                return ElPhysics::B(dNdx, loc);
            }

            /*********************RESULT ACCESS METHODS**************************/

            // ELASTICITY RESULTS

            /**
             * @brief Check if the element supports strain computation.
             * 
             * @return true if supported, false otherwise.
             */
            bool supports_strain()const override{
                return SupportsResult<ElPhysics,Tag_get_strain>::value;
            }

            /**
             * @brief Get the strain at a given location.
             * 
             * @param loc The local coordinates where the strain is to be computed.
             * @param ue The vector of nodal values.
             * @param dNdx The derivatives of the interpolation functions with respect to global coordinates.
             * @param D The constitutive matrix.
             * 
             * @return Vector The strain at the specified location.
             */
            Vector get_strain(  const Vector& loc, 
                                const Vector& ue,
                                const Matrix& dNdx,
                                const Matrix& D)const override{

                if constexpr(SupportsResult<ElPhysics,Tag_get_strain>::value){
                    return ElPhysics::compute(Tag_get_strain{},loc,ue,dNdx,D);
                }else{
                    error_message("get_strain()");
                }
                return Vector();
            }

            /**
             * @brief Check if the element supports stress computation.
             * 
             * @return true if supported, false otherwise.
             */
            bool supports_stress()const override{
                return SupportsResult<ElPhysics,Tag_get_stress>::value;
            }

            /**
             * @brief Get the stress at a given location.
             * 
             * @param loc The local coordinates where the stress is to be computed.
             * @param ue The vector of nodal values.
             * @param dNdx The derivatives of the interpolation functions with respect to global coordinates.
             * @param D The constitutive matrix.
             * 
             * @return Vector The stress at the specified location.
             */
            Vector get_stress(  const Vector& loc, 
                                const Vector& ue,
                                const Matrix& dNdx,
                                const Matrix& D)const override{

                if constexpr(SupportsResult<ElPhysics,Tag_get_stress>::value){
                    return ElPhysics::compute(Tag_get_stress{},loc,ue,dNdx,D);
                }else{
                    error_message("get_stress()");
                }
                return Vector();
            }

            /**
             * @brief Check if the element supports axial force computation.
             * 
             * @return true if supported, false otherwise.
             */
            bool supports_NX()const override{
                return SupportsResult<ElPhysics,Tag_Nx>::value;
            }

            /**
             * @brief Get the axial force at a given location.
             * 
             * @param loc The local coordinates where the axial force is to be computed.
             * @param ue The vector of nodal values.
             * @param dNdx The derivatives of the interpolation functions with respect to global coordinates.
             * @param D The constitutive matrix.
             * 
             * @return Vector The axial force at the specified location.
             */
            Vector get_NX(  const Vector& loc, 
                            const Vector& ue, 
                            const Matrix& dNdx,
                            const Matrix& D)const override{
                if constexpr(SupportsResult<ElPhysics,Tag_Nx>::value){
                    return ElPhysics::compute(Tag_Nx{},loc,ue,dNdx,D);
                }else{
                    error_message("get_NX()");
                }
                return Vector();
            }

            /**
             * @brief Check if the element supports shear force computation.
             * 
             * @return true if supported, false otherwise.
             */
            bool supports_VY()const override{
                return SupportsResult<ElPhysics,Tag_Vy>::value;
            }

            /**
             * @brief Get the shear force at a given location.
             * 
             * @param loc The local coordinates where the shear force is to be computed.
             * @param ue The vector of nodal values.
             * @param dNdx The derivatives of the interpolation functions with respect to global coordinates.
             * @param D The constitutive matrix.
             * 
             * @return Vector The shear force at the specified location.
             */
            Vector get_VY(  const Vector& loc, 
                            const Vector& ue, 
                            const Matrix& dNdx,
                            const Matrix& D)const override{
                if constexpr(SupportsResult<ElPhysics,Tag_Vy>::value){
                    return ElPhysics::compute(Tag_Vy{},loc,ue,dNdx,D);
                }else{
                    error_message("get_VY()");
                }
                return Vector();
            }
            

            /**
             * @brief Check if the element supports bending moment computation.
             * 
             * @return true if supported, false otherwise.
             */
            bool supports_MZ()const override{
                return SupportsResult<ElPhysics,Tag_Mz>::value;
            }

            /**
             * @brief Get the bending moment at a given location.
             * 
             * @param loc The local coordinates where the bending moment is to be computed.
             * @param ue The vector of nodal values.
             * @param dNdx The derivatives of the interpolation functions with respect to global coordinates.
             * @param D The constitutive matrix.
             * 
             * @return Vector The bending moment at the specified location.
             */
            Vector get_MZ(  const Vector& loc, 
                            const Vector& ue, 
                            const Matrix& dNdx,
                            const Matrix& D)const override{
                if constexpr(SupportsResult<ElPhysics,Tag_Mz>::value){
                    return ElPhysics::compute(Tag_Mz{},loc,ue,dNdx,D);
                }else{
                    error_message("get_MZ()");
                }
                return Vector();
            }
    };

} // namespace finelc
