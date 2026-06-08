#pragma once

#include <finelc/matrix.h>

#include <finelc/enumerations.h>

#include <finelc/mesh/mesh.h>
#include <finelc/analysis/interpolation.h>

#include <memory>
#include <vector>
#include <stdexcept>
#include <functional>
#include <variant>

#ifdef USE_PETSC
    #include <petscmat.h>
    #include <petscksp.h>
    #include <petscpc.h>
#endif

namespace finelc{

    using Evalfn = std::function<double(const Point&)>;

    struct IDMat{

        size_t rows;
        size_t cols;

        std::vector<DOFType> dofs;
        std::vector<int> data;

        IDMat(): rows(0), cols(0), data() {}

        IDMat(std::vector<DOFType> dofs_, size_t cols_):
        rows(dofs_.size()),
        cols(cols_),
        dofs(dofs_)
        {
            data = std::vector<int>(rows*cols, 0);
        }

        using iterator = std::vector<int>::iterator;
        using const_iterator = std::vector<int>::const_iterator;

        iterator begin(){ return data.begin(); }
        iterator end(){ return data.end(); }
        const_iterator begin() const{ return data.begin(); }
        const_iterator end() const{ return data.end(); }
        const_iterator cbegin() const{ return data.cbegin(); }
        const_iterator cend() const{ return data.cend(); }

        inline int operator()(int row, int col)const{
            return data[row + col*rows];
        }

        inline int operator()(DOFType d, int col)const{
            int i = 0;
            while(dofs[i] != d){
                i++;
                if(i>rows){
                    throw std::runtime_error("DOF not in ID matrix.");
                }
            }
            return data[i + col*rows];
        }

        inline int& operator()(DOFType d, int col){
            int i = 0;
            while(dofs[i] != d){
                i++;
                if(i>rows){
                    throw std::runtime_error("DOF not in ID matrix.");
                }
            }
            return data[i + col*rows];
        }


        inline int& operator()(int row, int col){
            return data[row + col*rows];
        }

    };

    struct BoundaryCondition{
        DOFType type;
        std::vector<int> nodes;
        double value;

        BoundaryCondition(DOFType type_, std::vector<int> nodes_, double value_):
            type(type_),
            nodes(nodes_),
            value(value_)
            {}

        BoundaryCondition(DOFType type_, int node_, double value_):
            type(type_),
            nodes({node_}),
            value(value_)
            {}
    };

    enum class ForceType: uint8_t{
        Nodal,
        Constant,
        Function
    };

    struct Force{

        using ForceEval = std::variant<double, Evalfn>;

        private:
            IGeometry_ptr domain;
            ForceType force_type;
            DOFType dof_type;
            int integration_points;

            ForceEval value;

            std::vector<int> nodes_or_elements;

            std::vector<int> nodes_from_mesh(const Mesh_ptr mesh);

        public:

            Force(DOFType dof_, IGeometry_ptr domain_, Evalfn function, int integration_points_=10):
                force_type(ForceType::Function), 
                dof_type(dof_), 
                domain(domain_), 
                value(function), 
                integration_points(integration_points_)
            {}

            Force(DOFType dof_, IGeometry_ptr domain_, double constant_, int integration_points_=10):
                force_type(ForceType::Constant), 
                dof_type(dof_), 
                domain(domain_), 
                value(constant_), 
                integration_points(integration_points_)
            {}

            Force(DOFType dof_, std::vector<int> nodes_, double value_):
                force_type(ForceType::Nodal), 
                dof_type(dof_), 
                nodes_or_elements(nodes_), 
                value(value_)
            {}

            Force(DOFType dof_, int node_, double value_):
                force_type(ForceType::Nodal), 
                dof_type(dof_), 
                nodes_or_elements({node_}),
                value(value_)
            {}

            

            void get_elements_from_geometry(const Mesh_ptr mesh);
            
            const std::vector<int>& get_nodes_or_elements()const{return nodes_or_elements;}
            ForceType get_type()const{return force_type;}
            DOFType get_dof_type()const{return dof_type;}

            double get_value_at(const Point& p)const;

            double get_integration_points()const{ return integration_points; }
            int get_dimension()const{ return domain->get_dimension(); }

    };

    class Analysis{

        private:

            Mesh_ptr mesh;
            IDMat ID;
            std::vector<BoundaryCondition> bcs;
            std::vector<Force> forces;
            int num_free_dofs;

            std::unique_ptr<IInterpolationScheme> interp;
            std::unique_ptr<std::vector<Triplet>> Kg_data = nullptr;
            std::unique_ptr<std::vector<Triplet>> Mg_data = nullptr;
            std::unique_ptr<Vector> fg_data = nullptr; 
            std::unique_ptr<Vector> fg_bc = nullptr; 
            std::unique_ptr<Vector> ug = nullptr; 
            std::unique_ptr<Vector> rho = nullptr;

            #ifdef USE_PETSC
                std::unique_ptr<Mat> K=nullptr;
                std::unique_ptr<Mat> M=nullptr;
                std::unique_ptr<PetscObjects> obj=nullptr;
            #endif
            
            std::vector<std::pair<DOFType,int>> get_dof_order(const IElement& el, int rows);

            void insert_triplets(   std::vector<Triplet>& triplets, 
                                    const std::vector<std::pair<DOFType,int>>& dof_order,
                                    const Matrix& Mat,
                                    int rows,
                                    double interp_const);

    
            enum class Assembler: uint8_t{Stiffness, Mass};

            Matrix Eigen_Matrix_from_triplets(const std::vector<Triplet>& triplets);
            std::vector<Triplet>  assemble(Assembler type);
            Vector assemble_fg();


        public:

            Analysis(
                Mesh_ptr mesh_,
                IDMat ID_,
                std::vector<BoundaryCondition> bcs_,
                std::vector<Force> forces_,
                int num_free_dofs_);

            Analysis(const Analysis& other)=delete;
            Analysis(Analysis&& other);

            ~Analysis();

            void destroy() noexcept;

            void set_ug(const Vector& u);

            Matrix Kg();
            Matrix Mg();
            const Vector& fg();

            void set_interpolation(InterpolationScheme name);
            void update_interpolation(InterpolationParameters identifier, const double value);
            void update_pseudo_density(double new_rho);
            void update_pseudo_density(const Vector& new_rho);

            InterpolationScheme get_interpolation_name()const;
            const Vector get_pseudo_density()const;
            Vector get_interpolation()const;
            Vector get_interpolation_derivative()const;

            Vector get_element_ue(const Vector& U, int el_number)const;
            Vector reconstruct_ug(const Vector& U)const;
            VectorNodes displaced_nodes(const Vector& U, double scale)const;

            int bc_size()const;
            int get_size()const;
            int total_size()const;
            int number_of_elements()const;

            IElement_ptr get_element(int el_number)const;

            Mesh_ptr get_mesh()const;
            const IDMat& get_ID()const;
            const VectorNodes& nodes()const;
            const VectorElements& elements()const;
            std::vector<int> get_free_dofs()const;
            std::vector<int> get_bc_dofs()const;

            


            void clear_forces();

            void add_force(DOFType dof, int node_number, double value);
            void add_force(DOFType dof, std::vector<int> nodes, double value);
            void add_force(DOFType dof, IGeometry_ptr geometry, double value, int integration_points=10);
            void add_force(DOFType dof, IGeometry_ptr geometry, Evalfn func, int integration_points=10);
            

            #ifdef USE_PETSC
                const Mat& get_PETSc_K();
                const Mat& get_PETSc_M();
                PetscObjects& get_PETSc_objects();
            #endif

    };

    using Analysis_ptr = std::shared_ptr<Analysis>;

    class AnalysisBuilder{

        protected:

            Mesh_ptr mesh;

            int num_free_dofs;
            std::vector<DOFType> unique_dofs;
            std::vector<BoundaryCondition> bc_vector;
            std::vector<Force> forces;
            IDMat ID;

            void get_degrees_of_freedom();
            void create_id_matrix();
            void populate_id_matrix();



        public:

            AnalysisBuilder(Mesh_ptr mesh_): mesh(mesh_){}
            ~AnalysisBuilder()=default;

            void add_boundary_condition(DOFType dof, int node_number, double value);
            void add_boundary_condition(DOFType dof, std::vector<int> node_number, double value);
            void add_boundary_condition(DOFType dof, IGeometry_ptr geometry, double value);

            void add_force(DOFType dof, int node_number, double value);
            void add_force(DOFType dof, std::vector<int> nodes, double value);
            void add_force(DOFType dof, IGeometry_ptr geometry, double value, int integration_points=10);
            void add_force(DOFType dof, IGeometry_ptr geometry, Evalfn func, int integration_points=10);

            Analysis build();
            

    };

} // namespace finelc