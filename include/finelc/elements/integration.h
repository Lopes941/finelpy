#pragma once

#include <finelc/matrix.h>
#include <finelc/enumerations.h>

#include <finelc/geometry/geometry.h>

#include <memory>
#include <vector>
#include <stdexcept>

namespace finelc{

    /**
     * @struct PointWeight
     * 
     * @brief Structure to hold a point and its associated weight for integration.
     */
    struct PointWeight{
        double weight; // The weight associated with the point
        Point point; // The point in space

        PointWeight(double x, double w):
        point(Point(x)),
        weight(w)
        {}

        PointWeight(Point p, double w):
        point(p),
        weight(w)
        {}
        
    };

    /**
     * @brief Convolve multiple PointWeight objects to create a higher-dimensional PointWeight.
     * 
     * @param p1 The first PointWeight object.
     * @param p2 The second PointWeight object.
     * @param p3 The third PointWeight object (optional, defaults to a weight of 1 at the origin).
     * 
     * @return PointWeight The resulting convolved PointWeight object.
     */
    PointWeight convolve_points(const PointWeight& p1, 
        const PointWeight& p2, 
        const PointWeight& p3=PointWeight(0,1));


    /**
     * @brief Compute the roots and weights of the Legendre polynomial of degree n.
     * 
     * @param n The degree of the Legendre polynomial.
     * @param tol The tolerance for convergence (default is 1e-14).
     * 
     * @return std::vector<PointWeight> A vector of PointWeight structures containing
     *  the roots and their corresponding weights.
     */
    std::vector<PointWeight> legendre_roots(int n, double tol = 1e-14);

    /**
     * @brief Get Gauss integration points and weights for a given geometry and dimension.
     * 
     * @param geometry The integration geometry (e.g., REGULAR, TRIANGLE).
     * @param num_pts The number of integration points.
     * @param dimensions The number of dimensions.
     * 
     * @return std::vector<PointWeight> A vector of PointWeight structures containing
     *  the integration points and their corresponding weights.
     */
    std::vector<PointWeight> get_gauss_points(
                            IntegrationGeometry geometry,
                            int num_pts, 
                            int dimensions);
    
    /**
     * @brief Template function to get Gauss integration points and weights for specific geometry and dimension.
     * 
     * @tparam Geo The integration geometry (template parameter).
     * @tparam dimension The number of dimensions (template parameter).
     * 
     * @param order The number of integration points.
     * 
     * @return std::vector<PointWeight> A vector of PointWeight structures containing
     *  the integration points and their corresponding weights.
     */
    template<IntegrationGeometry Geo, int dimension>
    std::vector<PointWeight> get_gauss_pair(int order){
        throw std::runtime_error("Invalid integration parameters");
    }

    /**
     * @brief Specialization of get_gauss_pair for REGULAR geometry in 1D.
     * 
     * @param num_pts The number of integration points.
     * 
     * @return std::vector<PointWeight> A vector of PointWeight structures containing
     *  the integration points and their corresponding weights.
     */
    template<>
    inline std::vector<PointWeight> get_gauss_pair<IntegrationGeometry::REGULAR, 1>(int num_pts){
        return legendre_roots(num_pts);
    }

    /**
     * @brief Specialization of get_gauss_pair for REGULAR geometry in 2D.
     * 
     * @param num_pts The number of integration points.
     * 
     * @return std::vector<PointWeight> A vector of PointWeight structures containing
     *  the integration points and their corresponding weights.
     */
    template<>
    inline std::vector<PointWeight> get_gauss_pair<IntegrationGeometry::REGULAR, 2>(int num_pts){
 
        std::vector<PointWeight> p1 = legendre_roots(num_pts);
        std::vector<PointWeight> p2 = p1;

        std::vector<PointWeight> p;
        p.reserve(p1.size()*p2.size());

        for(auto& pi: p1){
            for(auto& pj: p2){
                p.emplace_back(convolve_points(pi,pj));
            }
        }
        return p;
    }

    /**
     * @brief Specialization of get_gauss_pair for REGULAR geometry in 3D.
     * 
     * @param num_pts The number of integration points.
     * 
     * @return std::vector<PointWeight> A vector of PointWeight structures containing
     *  the integration points and their corresponding weights.
     */
    template<>
    inline std::vector<PointWeight> get_gauss_pair<IntegrationGeometry::REGULAR, 3>(int num_pts){

        std::vector<PointWeight> p1 = legendre_roots(num_pts);
        std::vector<PointWeight> p2 = p1;
        std::vector<PointWeight> p3 = p1;

        std::vector<PointWeight> p;
        p.reserve(p1.size()*p2.size()*p3.size());

        for(auto& pi: p1){
            for(auto& pj: p2){
                for(auto& pk: p3){
                    p.emplace_back(convolve_points(pi,pj,pk));
                }
            }
        }
        return p;
    }

    /**
     * @brief Convert a point from the reference triangle to the standard triangle.
     */
    inline Point convert_point(Point& p){
        return p*2. - 1.;
    }

    /**
     * @brief Specialization of get_gauss_pair for TRIANGLE geometry in 2D.
     * 
     * @param num_pts The number of integration points.
     * 
     * @return std::vector<PointWeight> A vector of PointWeight structures containing
     *  the integration points and their corresponding weights.
     */
    template<>
    inline std::vector<PointWeight> get_gauss_pair<IntegrationGeometry::TRIANGLE, 2>(int num_pts){

        // TODO
        std::vector<PointWeight> p;

        Point m(1./3.,1./3.);

        Point a[3]={
            Point(2./3.,1./6.),
            Point(1./6.,2./3.),
            Point(1./6.,1./6.)
        };

        Point b[3]={
            Point(0.6,0.2),
            Point(0.2,0.6),
            Point(0.2,0.2)
        };
        switch (num_pts){

            case 0:
                break;

            case 1:
                p = {
                    PointWeight(convert_point(m),2.),
                };
                break;
            
            case 2:
                p = {
                    PointWeight(convert_point(a[0]),2./3.),
                    PointWeight(convert_point(a[1]),2./3.),
                    PointWeight(convert_point(a[2]),2./3.),
                };
                break;

            case 3:
                p = {
                    PointWeight(convert_point(m),-27./24.),
                    PointWeight(convert_point(b[0]),25./24.),
                    PointWeight(convert_point(b[1]),25./24.),
                    PointWeight(convert_point(b[2]),25./24.),
                };
                break;
            
            default:
                std::cerr << num_pts << std::endl;
                throw std::runtime_error("Order of integration not programmed.");
                break;
        }

        return p;
    }

}