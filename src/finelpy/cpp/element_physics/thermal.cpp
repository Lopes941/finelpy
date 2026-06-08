#include <finelc/matrix.h>
#include <finelc/element_physics/thermal.h>

#include <array>
#include <vector>

namespace finelc{

    Matrix Thermal1DConduction::N(const Vector& N_geo, const Vector& loc){

        int num_nodes = N_geo.size();
        Matrix Nmat(1,num_nodes);
        Nmat.setZero();

        for(int i=0; i<num_nodes; i++){
            Nmat(0,i)   = N_geo(i);
        }

        return Nmat;

    }

    Matrix Thermal1DConduction::dNdx(const Matrix& dNdx_geo, const Vector& loc){
        return dNdx_geo;
    }

    Matrix Thermal1DConduction::B(const Matrix& dNdx,const Vector& loc){

        int num_nodes = dNdx.rows();
        Matrix B(1,num_nodes);
        B.setZero();

        for(int i=0; i<num_nodes; i++){
            B(0,i)   = dNdx(i,0);
        }
        return B;

    }

    Vector Thermal1DConduction::heat_flux(  const Vector& loc, 
                                const Vector& ue,
                                const Matrix& dNdx,
                                const Matrix& D){
        Vector q(3);
        Vector q1D = -D * B(dNdx, loc) * ue;

        q.setZero();
        q(0) = q1D(0);

        return q;
    }

    Vector Thermal1DConduction::compute( Tag_get_heat_flux, 
                    const Vector& loc, 
                    const Vector& ue,
                    const Matrix& dNdx,
                    const Matrix& D){
        return heat_flux(loc,ue,dNdx,D);
    }




    Matrix Thermal2DConduction::N(const Vector& N_geo, const Vector& loc){

        int num_nodes = N_geo.size();
        Matrix Nmat(2,num_nodes);
        Nmat.setZero();

        for(int i=0; i<num_nodes; i++){
            Nmat(0,i)   = N_geo(i);
            Nmat(1,i)   = N_geo(i);
        }

        return Nmat;

    }

    Matrix Thermal2DConduction::dNdx(const Matrix& dNdx_geo, const Vector& loc){
        return dNdx_geo;
    }

    Matrix Thermal2DConduction::B(const Matrix& dNdx,const Vector& loc){

        int num_nodes = dNdx.rows();
        Matrix B(2,num_nodes);
        B.setZero();

        for(int i=0; i<num_nodes; i++){
            B(0,i)   = dNdx(i,0);
            B(1,i)   = dNdx(i,1);
        }
        return B;

    }

    Vector Thermal2DConduction::heat_flux(  const Vector& loc, 
                                const Vector& ue,
                                const Matrix& dNdx,
                                const Matrix& D){
        Vector q(3);
        Vector q2D = -D * B(dNdx, loc) * ue;

        q.setZero();
        q(0) = q2D(0);
        q(1) = q2D(1);

        return q;
    }

    Vector Thermal2DConduction::compute( Tag_get_heat_flux, 
                    const Vector& loc, 
                    const Vector& ue,
                    const Matrix& dNdx,
                    const Matrix& D){
        return heat_flux(loc,ue,dNdx,D);
    }








    Matrix Thermal3DConduction::N(const Vector& N_geo, const Vector& loc){

        int num_nodes = N_geo.size();
        Matrix Nmat(3,num_nodes);
        Nmat.setZero();

        for(int i=0; i<num_nodes; i++){
            Nmat(0,i)   = N_geo(i);
            Nmat(1,i)   = N_geo(i);
            Nmat(2,i)   = N_geo(i);
        }

        return Nmat;

    }

    Matrix Thermal3DConduction::dNdx(const Matrix& dNdx_geo, const Vector& loc){
        return dNdx_geo;
    }

    Matrix Thermal3DConduction::B(const Matrix& dNdx,const Vector& loc){

        int num_nodes = dNdx.rows();
        Matrix B(3,num_nodes);
        B.setZero();

        for(int i=0; i<num_nodes; i++){
            B(0,i)   = dNdx(i,0);
            B(1,i)   = dNdx(i,1);
            B(2,i)   = dNdx(i,2);
        }
        return B;

    }

    Vector Thermal3DConduction::heat_flux(  const Vector& loc, 
                                const Vector& ue,
                                const Matrix& dNdx,
                                const Matrix& D){
        return -D * B(dNdx, loc) * ue;
    }

    Vector Thermal3DConduction::compute( Tag_get_heat_flux, 
                    const Vector& loc, 
                    const Vector& ue,
                    const Matrix& dNdx,
                    const Matrix& D){
        return heat_flux(loc,ue,dNdx,D);
    }
    

} // namespace finelc