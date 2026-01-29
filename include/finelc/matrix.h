#pragma once

#include <vector>
#include <stdexcept>
#include <iostream>
#include <optional>
#include <type_traits>

#ifdef USE_PETSC
    #include <petsc.h>
    using FinelIndex = PetscInt;
#else
    using FinelIndex = int;
#endif

#include <Eigen/Dense>
#include <Eigen/Sparse>


using Vector = Eigen::VectorXd;
using Triplet = Eigen::Triplet<double>;

using OptionalVector = std::optional<std::reference_wrapper<const Vector>>;

using DenseMatrix = Eigen::MatrixXd;
using SparseMatrix = Eigen::SparseMatrix<double, Eigen::RowMajor,FinelIndex>;

using DenseSolver = Eigen::PartialPivLU<DenseMatrix>;


using SparseSolver = Eigen::SparseLU<SparseMatrix>;
using IterativeSolver = Eigen::ConjugateGradient<SparseMatrix, Eigen::Lower|Eigen::Upper,SparseSolver>;



namespace finelc{

    inline Vector default_zero_vec(int size) {
        return Vector::Zero(size);
    }


    enum class MatrixType: uint8_t{Dense, Sparse};

    class NonZeroIterator{

        public:

            using DenseRef = const DenseMatrix*;
            using SparseRef = const SparseMatrix*;

            NonZeroIterator(): done_(true) {}

            NonZeroIterator(DenseRef dense, int c = 0, int r = 0)
                : dense_(dense), sparse_(nullptr), row_(r), col_(c), done_(false) {
                advance_dense();
            }

            NonZeroIterator(SparseRef sparse, int c = 0)
                : dense_(nullptr), sparse_(sparse), row_(0), col_(c), done_(false), it_(*sparse,c) {
                advance_sparse();
            }

            NonZeroIterator& operator++() {
                if (dense_) {
                    ++row_;
                    advance_dense();
                } else {
                    ++it_;
                    advance_sparse();
                }
                return *this;
            }

            bool operator==(const NonZeroIterator& other) const {
                if(done_ == other.done_) return true;
                return dense_ == other.dense_ &&
                    sparse_ == other.sparse_ &&
                    row_ == other.row_ &&
                    col_ == other.col_;
            }

            bool operator!=(const NonZeroIterator& other) const {
                return !(*this == other);
            }

            int row()const {return row_;}
            int col()const {return col_;}
            double value()const {return value_;}

            double& valueRef(){
                if(dense_){
                    return const_cast<DenseMatrix&>(*dense_)(row_,col_);
                }else{
                    return it_.valueRef();
                }
            }

        private:
            DenseRef dense_ = nullptr;
            SparseRef sparse_ = nullptr;

            SparseMatrix::InnerIterator it_;

            int row_ = 0, col_ = 0;
            double value_ = 0.0;
            bool done_ = false;

            void advance_dense(){
                int rows = dense_->rows();
                int cols = dense_->cols();
                while(col_<cols){
                    while(row_<rows){
                        value_ = (*dense_)(row_,col_);
                        if(value_!=0){
                            return;
                        }
                        row_++;
                    }
                    col_++;
                    row_ = 0;
                }
                done_ = true;
            }

            void advance_sparse() {
                while (col_ < sparse_->cols()) {
                    if (it_) {
                        value_ = it_.value();
                        row_ = it_.row();
                        return;
                    }
                    ++col_;
                    if (col_ < sparse_->cols())
                        it_ = SparseMatrix::InnerIterator(*sparse_,col_);
                }
                done_ = true;
            }

    };


    class Matrix{

        private:

            MatrixType type;

            union MatrixData{

                DenseMatrix dense;
                SparseMatrix sparse;

                MatrixData() {}
                ~MatrixData() {}
            } data;

            void destroy() noexcept;


        public:
        
            Matrix();

            Matrix(int rows, int cols, MatrixType type_=MatrixType::Dense);

            Matrix(const DenseMatrix& mat);
            Matrix(DenseMatrix&& mat);

            Matrix(const SparseMatrix& mat);
            Matrix(SparseMatrix&& mat);

            Matrix(const Matrix& other);
            Matrix(Matrix&& other);

            ~Matrix();

            Matrix& operator=(const Matrix& mat);
            Matrix& operator=(Matrix&& mat)noexcept;

            inline bool is_dense() const { return type==MatrixType::Dense;}
            inline bool is_sparse() const { return type==MatrixType::Sparse;}

            const DenseMatrix& get_dense_data()const;
            const SparseMatrix& get_sparse_data()const;

            DenseMatrix& get_mutable_dense_data();
            SparseMatrix& get_mutable_sparse_data();

            Matrix as_dense() const;
            Matrix as_sparse() const;

            double det() const;
            Matrix transpose()const;

            int rows() const;
            int cols() const;
            void setZero();

            double operator()(int row, int col) const;
            double& operator()(int row, int col);

            Matrix operator-()const;
            Matrix& operator+=(const Matrix& B);
            Matrix& operator-=(const Matrix& B);

            Matrix& operator*=(double scalar);
            Matrix& operator/=(double scalar);

            Vector get_col(int col) const;
            Vector get_row(int row) const;

            NonZeroIterator begin(int col=0)const{
                if(type==MatrixType::Dense){
                   return NonZeroIterator(&data.dense,col);
                }else{
                    return NonZeroIterator(&data.sparse,col);
                }
            }
            
            NonZeroIterator end() const {
                return NonZeroIterator();  // special "done" iterator
            }
            
            Matrix slice_by_column(const std::vector<int>& column_index) const;
            Matrix slice_by_column(int start, int end) const;

            void hstack(const Matrix& other_mat);
            void vstack(const Matrix& other_mat);

    };

    Matrix operator+(const Matrix& A, const Matrix& B);
    Matrix operator-(const Matrix& A, const Matrix& B);

    Vector operator*(const Matrix& A, const Vector& v);
    Vector operator*(const Vector& v, const Matrix& A);
    Matrix operator*(const Matrix& A, double scalar);
    Matrix operator*(double scalar, const Matrix& A);
    Matrix operator/(const Matrix& A, double scalar);
    Matrix operator*(const Matrix& A, const Matrix& B);

    Matrix hstack(const Matrix& A, const Matrix& B);
    Matrix vstack(const Matrix& A, const Matrix& B);

    std::ostream& operator<<(std::ostream& os, const Matrix& A);


    enum class SolverType: uint8_t{
        Direct,
        Iterative,
        };

    struct IterativeProperties{
        int max_iter;
        double tol;
    };

    #ifdef USE_PETSC
        struct PetscObjects{
            KSP ksp;
            PC pc;
            const Mat* Kmat;
            PetscInt n;
        };
    #endif

    class Solver{

        private:

            IterativeProperties prop;
            SolverType type;
            Matrix Mat;

            union SolverData{

                DenseSolver dense;
                SparseSolver sparse;
                IterativeSolver iterative;

                SolverData() {}
                ~SolverData() {}
            } solver;

            Vector dense_solver(const Vector& rhs);
            Matrix dense_solver(const DenseMatrix& rhs);

            Vector sparse_solver(const Vector& rhs);
            Matrix sparse_solver(const DenseMatrix& rhs);

        public:

            Solver(Matrix mat_obj, 
                SolverType type_=SolverType::Direct,
                IterativeProperties properties_=IterativeProperties{1000,1e-8});
            ~Solver()=default;

            Vector solve(const Vector& rhs);
            Matrix solve(const Matrix& rhs);
    };

    #ifdef USE_PETSC
    Vector petsc_solve_direct(const Vector& rhs,
                   PetscObjects& obj);
    #endif
    

} // namespace finelc

