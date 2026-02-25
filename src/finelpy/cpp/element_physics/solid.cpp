#include <finelc/matrix.h>
#include <finelc/element_physics/solid.h>

#include <array>
#include <vector>

namespace finelc{

    Matrix SolidStructural::N(const Vector& N_geo, const Vector& loc){

        int num_nodes = N_geo.size();
        Matrix Nmat(dof_per_node,dof_per_node*num_nodes);
        Nmat.setZero();

        for(int i=0; i<num_nodes; i++){
            Nmat(0,3*i)   = N_geo(i);
            Nmat(1,3*i+1) = N_geo(i);
            Nmat(2,3*i+2) = N_geo(i);
        }

        return Nmat;

    }

    Matrix SolidStructural::dNdx(const Matrix& dNdx_geo, const Vector& loc){
        return dNdx_geo;
    }

    Matrix SolidStructural::B(const Matrix& dNdx,const Vector& loc){

        int num_nodes = dNdx.rows();
        Matrix B(6,num_nodes*dof_per_node);
        B.setZero();

        for(int i=0; i<num_nodes; i++){
            B(0,3*i)   = dNdx(i,0);
            B(1,3*i+1) = dNdx(i,1);
            B(2,3*i+2) = dNdx(i,2);

            B(3,3*i)   = dNdx(i,1);
            B(3,3*i+1) = dNdx(i,0);

            B(4,3*i)   = dNdx(i,2);
            B(4,3*i+2) = dNdx(i,0);

            B(5,3*i+1) = dNdx(i,2);
            B(5,3*i+2) = dNdx(i,1);

        }
        return B;

    }

    Vector SolidStructural::strain(   const Vector& loc, 
                            const Vector& ue,
                            const Matrix& dNdx){
        return B(dNdx, loc) * ue;
    }

    Vector SolidStructural::stress(   const Vector& loc, 
                            const Vector& ue,
                            const Matrix& dNdx,
                            const Matrix& D){
        return D * strain(loc,ue,dNdx);
    }

    Vector SolidStructural::compute( Tag_get_strain, 
                    const Vector& loc, 
                    const Vector& ue,
                    const Matrix& dNdx,
                    const Matrix&){
        
        return strain(loc,ue,dNdx);

    }

    Vector SolidStructural::compute( Tag_get_stress, 
                    const Vector& loc, 
                    const Vector& ue,
                    const Matrix& dNdx,
                    const Matrix& D){
        return stress(loc,ue,dNdx,D);
    }

} // namespace finelc