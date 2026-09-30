#ifdef USE_PETSC

#include <finelc/matrix.h>

#include <vector>
#include <stdexcept>

#include <petsc.h>
#include <petscmat.h>
#include <petscksp.h>
#include <petscpc.h>

#include <numeric>
#include <algorithm>

#ifdef USE_SLEPC
    #include <slepc.h>
    #include <slepceps.h>
    #include <slepcst.h>
#endif

namespace finelc{

    Vector petsc_solve_direct(const Vector& rhs,
                   PetscObjects& obj){

        // RHS vector
        Vec b, x;
        PetscInt n = obj.n;
        VecCreateSeq(PETSC_COMM_SELF, n, &b);
        VecCreateSeq(PETSC_COMM_SELF, n, &x);

        // Index list
        std::vector<PetscInt> idx(n);
        std::iota(idx.begin(), idx.end(), 0);

        VecSetValues(b, n, idx.data(), rhs.data(), INSERT_VALUES);
        VecAssemblyBegin(b); VecAssemblyEnd(b);

        // Solver and preconditioner
        KSP& ksp = obj.ksp;
        PC& pc = obj.pc;
        
        KSPSolve(ksp, b, x);

        // Extract solution
        Vector sol(n);
        VecGetValues(x, n, idx.data(), sol.data());

        VecDestroy(&b);
        VecDestroy(&x);

        return sol;
    }

    // Vector petsc_solve_iterative(const Vector& rhs,
    //                const IterativeProperties &prop,
    //                const Matrix *Mat,
    //                 PETSC_METHOD method){

    //     const SparseMatrix& M = Mat->get_sparse_data();
    //     Mat A = eigen_to_petsc(M);

    //     PetscInt n = M.rows();

    //     // RHS vector
    //     Vec b, x;
    //     VecCreateSeq(PETSC_COMM_SELF, n, &b);
    //     VecCreateSeq(PETSC_COMM_SELF, n, &x);

    //     // Index list
    //     std::vector<PetscInt> idx(n);
    //     std::iota(idx.begin(), idx.end(), 0);

    //     VecSetValues(b, n, idx.data(), rhs.data(), INSERT_VALUES);
    //     VecAssemblyBegin(b); VecAssemblyEnd(b);

    //     // KSP solver
    //     KSP ksp;
    //     KSPCreate(PETSC_COMM_SELF, &ksp);
    //     KSPSetOperators(ksp, A, A);

    //     // Solver type: CG
    //     if (method==PETSC_METHOD::CG){
    //         KSPSetType(ksp, KSPCG);
    //     }

    //     // Preconditioner: ILU
    //     PC pc;
    //     KSPGetPC(ksp, &pc);
    //     PCSetType(pc, PCILU);

    //     // Tolerances
    //     KSPSetTolerances(ksp,
    //         prop.tol,
    //         PETSC_DEFAULT,
    //         PETSC_DEFAULT,
    //         prop.max_iter
    //     );

    //     KSPSetFromOptions(ksp);
    //     KSPSolve(ksp, b, x);

    //     // Extract solution
    //     Vector sol(n);
    //     VecGetValues(x, n, idx.data(), sol.data());

    //     VecDestroy(&b);
    //     VecDestroy(&x);
    //     MatDestroy(&A);
    //     KSPDestroy(&ksp);

    //     return sol;
    //}

    #ifdef USE_SLEPC
    
        std::vector<EigenPair> slepc_solve_eigen(SlepcObjects& obj, int k, double sigma){

            // Generalized eigenvalue operators
            EPSSetOperators(obj.eps, *obj.Kmat, *obj.Mmat);
            EPSSetProblemType(obj.eps, EPS_GHEP);

            EPSSetTarget(obj.eps, sigma);
            EPSSetWhichEigenpairs(obj.eps, EPS_TARGET_MAGNITUDE);
            EPSSetDimensions(obj.eps, k, PETSC_DECIDE, PETSC_DECIDE);


            ST st;
            EPSGetST(obj.eps, &st);
            STSetType(st, STSINVERT);
            STSetShift(st, sigma);
            STSetMatStructure(st, SAME_NONZERO_PATTERN);

            EPSSetFromOptions(obj.eps);

            EPSSolve(obj.eps);

            PetscInt nconv = 0;
            EPSGetConverged(obj.eps, &nconv);

            if (nconv == 0) {
                throw std::runtime_error("SLEPc Error: No eigenvalues converged.");
            }

            std::vector<EigenPair> pairs;
            int output_count = std::min(static_cast<int>(nconv), k);
            pairs.reserve(output_count);

            Vec vr, vi;
            MatCreateVecs(*obj.Kmat, &vr, &vi);

            PetscInt n;
            VecGetSize(vr, &n);
            std::vector<PetscInt> idx(n);
            std::iota(idx.begin(), idx.end(), 0);

            for (int i = 0; i < output_count; ++i) {
                PetscScalar kr, ki;
                EPSGetEigenpair(obj.eps, i, &kr, &ki, vr, vi);

                EigenPair pair;
                pair.val = PetscRealPart(kr);
                pair.vector.resize(n);
                VecGetValues(vr, n, idx.data(), pair.vector.data());

                pairs.push_back(std::move(pair));
            }

            VecDestroy(&vr);
            VecDestroy(&vi);

            return pairs;
        }

    #endif


    
} // namespace finelc

#endif