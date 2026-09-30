
#include <finelc/matrix.h>

#include <vector>
#include <stdexcept>
#include <string>
#include <variant>

#include <iostream>

#include <Spectra/MatOp/DenseSymMatProd.h>
#include <Spectra/MatOp/SparseCholesky.h>
#include <Spectra/SymGEigsSolver.h>


namespace finelc{


    Vector default_iterative_sparse_solver(IterativeSolver& iterative_solver, 
                                            const Vector& rhs, 
                                            const IterativeProperties& prop){
            
        iterative_solver.setMaxIterations(prop.max_iter);
        iterative_solver.setTolerance(prop.tol);
        return iterative_solver.solve(rhs);
    }

    Matrix default_iterative_sparse_solver(IterativeSolver& iterative_solver, 
                                            const DenseMatrix& rhs, 
                                            const IterativeProperties& prop){
            
        iterative_solver.setMaxIterations(prop.max_iter);
        iterative_solver.setTolerance(prop.tol);
        return Matrix(iterative_solver.solve(rhs));
    }


    Vector default_direct_sparse_solver(SparseSolver& sparse_solver, const Vector& rhs, const Matrix& Mat){
        sparse_solver.factorize(Mat.get_sparse_data());
        return sparse_solver.solve(rhs);
    }

    Matrix default_direct_sparse_solver(SparseSolver& sparse_solver, const DenseMatrix& rhs, const Matrix& Mat){
        sparse_solver.factorize(Mat.get_sparse_data());
        return Matrix(sparse_solver.solve(rhs));
    }

    Vector Solver::dense_solver(const Vector& rhs){
        return solver.dense.solve(rhs);
    }

    Matrix Solver::dense_solver(const DenseMatrix& rhs){
        
        return Matrix(solver.dense.solve(rhs));
    }

    Vector Solver::sparse_solver(const Vector& rhs){
        if(type == SolverType::Iterative){
            return default_iterative_sparse_solver(solver.iterative,rhs,prop);
        }else{
            return default_direct_sparse_solver(solver.sparse,rhs,Mat);
        }
    }

    Matrix Solver::sparse_solver(const DenseMatrix& rhs){

        if(type == SolverType::Iterative){

            return default_iterative_sparse_solver(solver.iterative,rhs,prop);

        }else{
            return default_direct_sparse_solver(solver.sparse,rhs,Mat);
        }
    }



    Solver::Solver(Matrix mat_obj, SolverType type_, IterativeProperties properties_): 
    Mat(mat_obj), type(type_), prop(properties_)
    {
        if(Mat.is_dense()){
            new (&solver.dense) DenseSolver();
            solver.dense.compute(Mat.get_dense_data());
        }else{
            if(type == SolverType::Iterative){
                new (&solver.iterative) IterativeSolver();
                solver.iterative.compute(Mat.get_sparse_data());

            }else{
                new (&solver.sparse) SparseSolver();
                solver.sparse.analyzePattern(Mat.get_sparse_data());
            }
        }
    }

    Vector Solver::solve(const Vector& rhs){
        if(Mat.is_dense()){
            return dense_solver(rhs);
        }else{
            return sparse_solver(rhs);
        }
    }

    Matrix Solver::solve(const Matrix& rhs){

        if(rhs.is_sparse()){
            return solve(rhs.as_dense());
        }

        if(Mat.is_dense()){
            return dense_solver(rhs.get_dense_data());
        }else{
            return sparse_solver(rhs.get_dense_data());
        }
    }


    GenEigen::GenEigen(Matrix A_obj, Matrix B_obj, int k, double sigma, IterativeProperties properties_): 
    A(A_obj), B(B_obj), sigma(sigma), k(k), prop(properties_)
    {}

    std::vector<EigenPair> GenEigen::solve(){

        Matrix A_sparse_mat = A.is_sparse() ? A : A.as_sparse();
        Matrix B_sparse_mat = B.is_sparse() ? B : B.as_sparse();

        const auto& A_sp = A_sparse_mat.get_sparse_data();
        const auto& B_sp = B_sparse_mat.get_sparse_data();

        // 2. Set up Spectra operators
        Spectra::SparseSymMatProd<double, Eigen::RowMajor, Eigen::Lower, FinelIndex> op(A_sp);
        Spectra::SparseCholesky<double, Eigen::RowMajor, Eigen::Lower, FinelIndex> Bop(B_sp);

        int ncv = std::min(static_cast<int>(A_sp.rows()), std::max(2 * k, k + 2));

        Spectra::SymGEigsSolver<decltype(op), 
                                decltype(Bop), 
                                Spectra::GEigsMode::Cholesky> 
                                geigs(op, Bop, k, ncv);

        // 3. Initialize workspace and solve
        geigs.init();
        int nconv = geigs.compute(Spectra::SortRule::SmallestMagn, prop.max_iter, prop.tol);

        if (geigs.info() != Spectra::CompInfo::Successful || nconv == 0) {
            throw std::runtime_error("Spectra eigenvalue computation failed to converge.");
        }

        // 4. Return the primary eigenvalue and eigenvector pair
        std::vector<EigenPair> pairs(k);
        for (int i = 0; i < k; ++i) {
            pairs[i].val = geigs.eigenvalues()(i);
            pairs[i].vector = geigs.eigenvectors().col(i);
        }
        return pairs;
            
    }
    
} // namespace finelc
