#include <finelc/matrix.h>

#include <finelc/analysis/analysis.h>
#include <finelc/solver/solver.h>
#include <finelc/result/result.h>

#include <optional>

namespace finelc{

    void StaticSolver::default_solver(){
        /*if(analysis->get_size() > 1'000'000){
            type = std::make_unique<SolverType>(SolverType::Iterative);
        }else{*/
            type = std::make_unique<SolverType>(SolverType::Direct);
       // }
    }

    StaticResult StaticSolver::solve(){

        #ifdef USE_PETSC
            PetscObjects& obj = analysis->get_PETSc_objects();
            obj.Kmat = &analysis->get_PETSc_K();
            KSPSetOperators(obj.ksp, *obj.Kmat, *obj.Kmat);
            KSPSetReusePreconditioner(obj.ksp,PETSC_FALSE);
            PCFactorSetReuseOrdering(obj.pc, PETSC_FALSE);
            PCFactorSetReuseFill(obj.pc, PETSC_FALSE);
            Vector u = petsc_solve_direct(
                analysis->fg(), 
                obj);
            return StaticResult(std::move(u),analysis); 
        #else

        if(!type) default_solver();

        if(!solver){
            Matrix Kg = analysis->Kg();
            solver = std::make_unique<Solver>(Kg,*type);
        }

        const Vector& fg = analysis->fg();
        Vector u = solver->solve(fg);
        return StaticResult(std::move(u),analysis);
        #endif

    }

    EigenResult EigenvalueSolver::solve(){

        #ifdef USE_SLEPC

            SlepcObjects& obj = analysis->get_SLEPc_objects();
            obj.Kmat = &analysis->get_PETSc_K();
            obj.Mmat = &analysis->get_PETSc_M();

            std::vector<EigenPair> pairs = slepc_solve_eigen(
                analysis->get_SLEPc_objects(),
                k,0);

            return EigenResult(std::move(pairs),analysis);

        #else

        if(!eigen_solver){
            Matrix Kg = analysis->Kg();
            Matrix Mg = analysis->Mg();
            eigen_solver = std::make_unique<GenEigen>(Kg,Mg,k,0);
        }

        std::vector<EigenPair> pairs = eigen_solver->solve();
        return EigenResult(pairs);
        #endif
    }

} // namespace finelc

