#include <finelc/matrix.h>
#include <finelc/element_geometry/hex.h>

#include <vector>
#include <iostream>

namespace finelc{

    Vector Hex8::N(const Vector& loc){

        double xi = loc(0);
        double eta = loc(1);
        double zeta = loc(2);

        Vector N(8);

        N(0) = 0.125*(1-xi)*(1-eta)*(1-zeta);
        N(1) = 0.125*(1+xi)*(1-eta)*(1-zeta);
        N(2) = 0.125*(1+xi)*(1+eta)*(1-zeta);
        N(3) = 0.125*(1-xi)*(1+eta)*(1-zeta);
        N(4) = 0.125*(1-xi)*(1-eta)*(1+zeta);
        N(5) = 0.125*(1+xi)*(1-eta)*(1+zeta);
        N(6) = 0.125*(1+xi)*(1+eta)*(1+zeta);
        N(7) = 0.125*(1-xi)*(1+eta)*(1+zeta);

        return N;

    }

    Matrix Hex8::dNdxi(const Vector& loc){

        double xi = loc(0);
        double eta = loc(1);
        double zeta = loc(2);

        DenseMatrix dNdxi(3,8);

        dNdxi(0,0) = -0.125*(1-eta)*(1-zeta);
        dNdxi(0,1) =  0.125*(1-eta)*(1-zeta);
        dNdxi(0,2) =  0.125*(1+eta)*(1-zeta);
        dNdxi(0,3) = -0.125*(1+eta)*(1-zeta);
        dNdxi(0,4) = -0.125*(1-eta)*(1+zeta);
        dNdxi(0,5) =  0.125*(1-eta)*(1+zeta);
        dNdxi(0,6) =  0.125*(1+eta)*(1+zeta);
        dNdxi(0,7) = -0.125*(1+eta)*(1+zeta);

        dNdxi(1,0) =  -0.125*(1-xi)*(1-zeta);
        dNdxi(1,1) =  -0.125*(1+xi)*(1-zeta);
        dNdxi(1,2) =  0.125*(1+xi)*(1-zeta);
        dNdxi(1,3) =  0.125*(1-xi)*(1-zeta);
        dNdxi(1,4) =  -0.125*(1-xi)*(1+zeta);
        dNdxi(1,5) =  -0.125*(1+xi)*(1+zeta);
        dNdxi(1,6) =  0.125*(1+xi)*(1+zeta);
        dNdxi(1,7) =  0.125*(1-xi)*(1+zeta);

        dNdxi(2,0) =  -0.125*(1-xi)*(1-eta);
        dNdxi(2,1) =  -0.125*(1+xi)*(1-eta);
        dNdxi(2,2) =  -0.125*(1+xi)*(1+eta);
        dNdxi(2,3) =  -0.125*(1-xi)*(1+eta);
        dNdxi(2,4) =  0.125*(1-xi)*(1-eta);
        dNdxi(2,5) =  0.125*(1+xi)*(1-eta);
        dNdxi(2,6) =  0.125*(1+xi)*(1+eta);
        dNdxi(2,7) =  0.125*(1-xi)*(1+eta);

        return dNdxi;

    }

    Matrix Hex8::dNdx( const VectorNodes& element_nodes,
                        const Vector& loc){

        Matrix dN_dxi = dNdxi(loc);
        Matrix Jacobian = J(element_nodes, loc);

        return Solver(Jacobian).solve(dN_dxi).transpose();

    }

    Matrix Hex8::J(const VectorNodes& element_nodes,
            const Vector& loc){


        Matrix dN_dxi = dNdxi(loc);

        Matrix cds(8,3);

        for(int nd=0; nd<8; nd++){
            cds(nd,0) = element_nodes[nd]->x;
            cds(nd,1) = element_nodes[nd]->y;
            cds(nd,2) = element_nodes[nd]->z;
        }

        Matrix J = (dN_dxi*cds).transpose();

        return J;
    }

    double Hex8::detJ(const VectorNodes& element_nodes,
                const Vector& loc){
        return J(element_nodes,loc).det();
    }


} // namespace finel