
#include <finelc/enumerations.h>

#include <finelc/matrix.h>
#include <finelc/analysis/analysis.h>
#include <finelc/analysis/interpolation.h>

#include <stdexcept>
#include <unordered_set>
#include <memory>

namespace finelc{

    std::unique_ptr<IInterpolationScheme> create_interpolation(InterpolationScheme name = InterpolationScheme::NONE){

        switch (name)
        {
        case InterpolationScheme::NONE:
            return std::make_unique<InterpolationSchemeAdapter<NoInterpolation>>();
            break;
        
        case InterpolationScheme::SIMP:
            return std::make_unique<InterpolationSchemeAdapter<SIMPInterpolationScheme>>();
            break;
        
        default:
            throw std::runtime_error("Invalid interpolation scheme given.");
            break;
        }

    }


    std::vector<int> get_mesh_nodes_in_geometry(const Mesh_ptr mesh, const IGeometry_ptr geo){

        const VectorNodes& mesh_nodes = mesh->get_nodes();

        std::vector<int> nodes;
        for(int nd=0; nd<mesh_nodes.size(); nd++){
            if(geo->is_inside(*mesh_nodes[nd])){
                nodes.emplace_back(nd);
            }
        }
        return nodes;

    }


    // ================================================================
    // Force addition functions
    // =================================================================


    void create_force(
        std::vector<Force>& forces, 
        DOFType dof, 
        int node_number, 
        double value){

        forces.push_back(Force(dof,node_number,value));
    }

    void create_force(
        std::vector<Force>& forces, 
        DOFType dof, 
        std::vector<int> nodes, 
        double value){

        for(auto& node_number : nodes){
            forces.push_back(Force(dof,node_number,value));
        }
    }

    void create_force(
        std::vector<Force>& forces, 
        const Mesh_ptr mesh,
        DOFType dof, 
        IGeometry_ptr geometry, 
        double value, 
        int integration_points){

        Force force(dof,geometry,value,integration_points);
        force.get_elements_from_geometry(mesh);
        forces.push_back(force);

    }

    void create_force(
        std::vector<Force>& forces, 
        const Mesh_ptr mesh,
        DOFType dof, 
        IGeometry_ptr geometry, 
        Evalfn func, 
        int integration_points){

        Force force(dof,geometry,func,integration_points);
        force.get_elements_from_geometry(mesh);
        forces.push_back(force);
        
    }


    // ================================================================
    // Class: Force
    // Manages the forces, integrating them in elements
    // =================================================================


    std::vector<int> Force::nodes_from_mesh(const Mesh_ptr mesh){
        return get_mesh_nodes_in_geometry(mesh, domain);
    }

    void Force::get_elements_from_geometry(const Mesh_ptr mesh){
                
        std::vector<int> force_nodes = nodes_from_mesh(mesh);
        nodes_or_elements.clear();

        for(int el_number=0; el_number<mesh->number_of_elements(); el_number++){
            const IElement_ptr el = mesh->get_element(el_number);
            const std::vector<int>& el_nodes = el->get_node_numbering();
            for(const auto& node: el_nodes){
                if(std::find(force_nodes.begin(), force_nodes.end(), node) != force_nodes.end()){
                    nodes_or_elements.emplace_back(el_number);
                    break;
                }
            }
        }
    }

    double Force::get_value_at(const Point& p)const{
        if(force_type == ForceType::Nodal || force_type == ForceType::Constant){
            return std::get<double>(value);
        }else{
            if(domain->is_inside(p)){
                return std::get<Evalfn>(value)(p);
            }else{
                return 0;
            }
            
        }
    }


    // ================================================================
    // Class: Analysis
    // Manages the force, boundary conditions and degrees of freedom.
    // =================================================================


    Analysis::Analysis( Mesh_ptr mesh_,
                        IDMat ID_,
                        std::vector<BoundaryCondition> bcs_,
                        std::vector<Force> forces_,
                        int num_free_dofs_):
        mesh(mesh_),ID(ID_),bcs(bcs_),
        forces(forces_),
        num_free_dofs(num_free_dofs_),
        interp(create_interpolation())
        {}

    Analysis::Analysis(Analysis&& other){

        mesh = std::move(other.mesh);
        ID = std::move(other.ID);
        bcs = std::move(other.bcs);
        forces = std::move(other.forces);
        interp = std::move(other.interp);
        num_free_dofs = other.num_free_dofs;

        Kg_data = std::move(other.Kg_data);
        Mg_data = std::move(other.Mg_data);
        fg_data = std::move(other.fg_data);
        fg_bc   = std::move(other.fg_bc);

        #ifdef USE_PETSC
            K = std::move(other.K);
            M = std::move(other.M);
            obj = std::move(other.obj);
        #endif
    }

    Analysis::~Analysis(){
    }

    void Analysis::destroy() noexcept{
        #ifdef USE_PETSC
            if(obj){
                KSPDestroy(&obj->ksp);
            }
            obj = nullptr;
            if(K) MatDestroy(&(*K));
            K = nullptr;
            if(M) MatDestroy(&(*M));
            M = nullptr;
        #endif
    }

    std::vector<std::pair<DOFType,int>> Analysis::get_dof_order(const IElement& el, int rows){

        const std::vector<int>& node_numbering = el.get_node_numbering();
        std::vector<DOFType> dofs = el.dofs();

        std::vector<std::pair<DOFType,int>> dof_order;
        dof_order.reserve(rows);
        for(auto& node: node_numbering){
            for(auto& dof: dofs){
                dof_order.emplace_back(std::make_pair(dof,node));
            }
        }
        return dof_order;
    }

    void Analysis::insert_triplets( std::vector<Triplet>& triplets, 
                                    const std::vector<std::pair<DOFType,int>>& dof_order,
                                    const Matrix& Mat,
                                    int rows,
                                    double interp_const){

        for(int i=0; i<rows; i++){
                auto& pair_i = dof_order[i];
                int dofi = ID(pair_i.first,pair_i.second);
                if(dofi<0) continue;
                for(int j=0; j<rows; j++){
                    auto& pair_j = dof_order[j];
                    int dofj = ID(pair_j.first,pair_j.second);
                    if(dofj>=0){
                        triplets.push_back(Triplet(dofi,dofj,Mat(i,j)*interp_const));
                    }else{
                        double bc_val = bcs[-dofj-1].value;
                        if(bc_val == 0) continue;
                        (*fg_bc)(dofi) -= bc_val*Mat(i,j)*interp_const;
                    }
                }
                
            }
    }

    std::vector<Triplet> Analysis::assemble(Assembler type){

        // TODO insert get ue logic here
        OptionalVector ue = std::nullopt;


        int non_zero_vals = 0;
        const VectorElements& elements = mesh->get_elements();
        for(auto& el: elements){
            non_zero_vals += el->displacement_size()*el->displacement_size();
        }

        using ElementMatrixFn = const Matrix& (IElement::*)(OptionalVector);
        ElementMatrixFn getMatrix = nullptr;

        switch (type) {
            case Assembler::Stiffness:
                getMatrix = &IElement::Ke;
                fg_bc = std::make_unique<Vector>(num_free_dofs);
                fg_bc->setZero();
                break;
            case Assembler::Mass:
                getMatrix = &IElement::Me;
                break;
        }

        std::vector<Triplet> triplets;
        triplets.reserve(non_zero_vals);

        Vector interps;
        bool has_interp = interp->name() != InterpolationScheme::NONE;
        if(has_interp){
            interps = interp->apply(*rho);
        }else{
            interps = Vector::Ones(1);
        }

        for(int el=0; el<elements.size(); el++){

            IElement& element = *elements[el];
            const Matrix& Mat_e = ((element).*getMatrix)(ue);
            int rows = Mat_e.rows();

            double rho_el = has_interp? interps(el):interps(0);

            std::vector<std::pair<DOFType,int>> dof_order = get_dof_order(element,rows);

            insert_triplets(triplets, dof_order,Mat_e, rows, rho_el);
        }

        return triplets;
    
    }

    Matrix Analysis::Eigen_Matrix_from_triplets(const std::vector<Triplet>& triplets){
        
        SparseMatrix Mat(num_free_dofs,num_free_dofs);
        Mat.setFromTriplets(triplets.begin(), triplets.end());
        return Matrix(Mat);
    }


    Vector Analysis::assemble_fg(){

        // TODO insert get ue logic here
        OptionalVector ue = std::nullopt;

        Vector fg = *fg_bc;

        for(auto& force : forces){

            if(force.get_type() == ForceType::Nodal){

                const std::vector<int>& nodes = force.get_nodes_or_elements();
                DOFType dof = force.get_dof_type();
                double val = force.get_value_at(Point());
                
                for(auto& node : nodes){
                    int loc = ID(dof,node);
                    if(loc>=0){
                        fg(loc) += val;
                    }
                }

            }else if (force.get_type() == ForceType::Function ||
                       force.get_type() == ForceType::Constant){

                const std::vector<int>& elements = force.get_nodes_or_elements();
                DOFType dof = force.get_dof_type();
                for(auto& el_number : elements){

                    IElement_ptr el = get_element(el_number);
                    const std::vector<int> node_numbering = el->get_node_numbering();

                    std::vector<PointWeight> gauss_pts = el->integration_pair(
                        force.get_integration_points(),
                        force.get_dimension());


                    int size = el->number_of_nodes()*el->dofs_per_node();
                    int dofs_per_node = el->dofs_per_node();
                    Vector fe(size);
                    fe.setZero();

                    std::vector<DOFType> eldofs = el->dofs();
                    auto it = std::find(eldofs.begin(),eldofs.end(), dof);
                    if(it == eldofs.end()) continue;
                    int dof_number = std::distance(eldofs.begin(), it);
                    
                    for(auto& gp : gauss_pts){
                        Vector gp_vec = gp.point.as_vector();
                        Vector N = el->N(gp_vec,std::nullopt).get_row(dof_number);
                        double double_val = force.get_value_at(el->local_to_global(gp.point));
                        double detJ = el->detJ(gp_vec);
                        fe += N*double_val*detJ*gp.weight;
                    }

                    int k=0;
                    for(auto& node : node_numbering){
                        for(auto& eldof: eldofs){
                            int loc = ID(eldof,node);
                            if(loc>=0){
                                fg(loc) += fe(k);
                            }
                            k++;
                        }
                    }
                }

            }
        }
        

        return fg;
    }

    void Analysis::set_ug(const Vector& u){
        Kg_data = nullptr;
        Mg_data = nullptr;
        fg_data = nullptr;
        fg_bc   = nullptr;
        ug      = std::make_unique<Vector>(u);
    }

    Matrix Analysis::Kg(){

        if(!Kg_data){
            Kg_data = std::make_unique<std::vector<Triplet>>(assemble(Assembler::Stiffness));
        }
        return Eigen_Matrix_from_triplets(*Kg_data);

    }

    Matrix Analysis::Mg(){

        if(!Mg_data){
            Mg_data = std::make_unique<std::vector<Triplet>>(assemble(Assembler::Mass));
        }
        return Eigen_Matrix_from_triplets(*Mg_data);
    }

    const Vector& Analysis::fg(){

        if(!fg_data){
            if(!fg_bc){
                Kg_data = std::make_unique<std::vector<Triplet>>(assemble(Assembler::Stiffness));
            }
            fg_data = std::make_unique<Vector>(assemble_fg());
        }
        return *fg_data;

    }

    void Analysis::set_interpolation(InterpolationScheme name){
        interp = create_interpolation(name);
        if(interp->name()==InterpolationScheme::NONE){
            rho = nullptr;
        }else{
            rho = std::make_unique<Vector>();
            *rho = Vector::Ones(mesh->number_of_elements());
        }
    }

    void Analysis::update_interpolation(InterpolationParameters identifier, const double value){
        interp->update_property(identifier,value);
    }

    void Analysis::update_pseudo_density(double new_rho){
        if(!rho){throw std::runtime_error("Pseudo-density vector not initialized.");}
        *rho = Vector::Constant(mesh->number_of_elements(),new_rho);
        Kg_data = nullptr;
        Mg_data = nullptr;
        fg_data = nullptr;
    }

    void Analysis::update_pseudo_density(const Vector& new_rho){
        if(!rho){throw std::runtime_error("Pseudo-density vector not initialized.");}
        *rho = new_rho;
        Kg_data = nullptr;
        Mg_data = nullptr;
        fg_data = nullptr;
    }

    InterpolationScheme Analysis::get_interpolation_name()const{
        return interp->name();
    }

    const Vector Analysis::get_pseudo_density()const{
        if(!rho) 
            return Vector::Ones(mesh->number_of_elements()); 
        return *rho;
    }

    Vector Analysis::get_interpolation()const{
        return interp->apply(get_pseudo_density());
    }
    
    Vector Analysis::get_interpolation_derivative()const{
        return interp->derivative(get_pseudo_density());
    }

    Vector Analysis::get_element_ue(const Vector& U, int el_number)const{

        IElement_ptr el = get_element(el_number);
        std::vector<DOFType> el_dofs = el->dofs();
        const std::vector<int> node_numbering = el->get_node_numbering();

        int ue_size = el->displacement_size();
        Vector ue(ue_size);

        
        int k=0;
        for(auto& node: node_numbering){
            for(auto& dof: el_dofs){
                int loc = ID(dof,node);
                if(loc >= 0){
                    ue(k) = U(loc);
                }else{
                    ue(k) = bcs[-loc-1].value;
                }
                k++;
            }
        }
        return ue;
    }

    Vector Analysis::reconstruct_ug(const Vector& U)const{

        Vector ug(total_size());
        int k=0;
        for(int node=0; node<ID.cols; node++){
            for (int dof=0; dof<ID.rows; dof++){
                int loc = ID(dof,node);
                if(loc>=0){
                    ug(k) = U(loc);
                }else{
                    ug(k) = bcs[-loc-1].value;
                }
                k++;
            }
        }
        return ug;
    }

    VectorNodes Analysis::displaced_nodes(const Vector& U, double scale)const{

        bool has_x, has_y, has_z;
        const std::vector<DOFType>& dofs = ID.dofs;

        if(std::find(dofs.begin(), dofs.end(), DOFType::UX) != dofs.end()){
            has_x = true;
        }else{
            has_x = false;
        }

        if(std::find(dofs.begin(), dofs.end(), DOFType::UY) != dofs.end()){
            has_y = true;
        }else{
            has_y = false;
        }

        if(std::find(dofs.begin(), dofs.end(), DOFType::UZ) != dofs.end()){
            has_z = true;
        }else{
            has_z = false;
        }

        VectorNodes nodes;
        nodes.reserve(mesh->number_of_nodes());

        for(size_t node=0; node<mesh->number_of_nodes();node++){

            const Node_ptr mesh_node = mesh->get_node(node);

            int dofs_x=0;
            int dofs_y=0;
            int dofs_z=0;

            double u_x = mesh_node->x;
            double u_y = mesh_node->y;
            double u_z = mesh_node->z;
            

            if(has_x){
                dofs_x = ID(DOFType::UX,node);
                if(dofs_x>=0){
                    u_x += U(dofs_x)*scale ;
                }else{
                    u_x += bcs[(-dofs_x-1)].value*scale;
                }
            }


            if(has_y){
                dofs_y = ID(DOFType::UY,node);
                if(dofs_y>=0){
                    u_y += U(dofs_y)*scale;
                }else{
                    u_y += bcs[(-dofs_y-1)].value*scale;
                }
            }


            if(has_z){
                dofs_z = ID(DOFType::UZ,node);
                if(dofs_z>=0){
                    u_z += U(dofs_z)*scale;
                }else{
                    u_z += bcs[(-dofs_z-1)].value*scale;
                }
            }

            Node_ptr nd = std::make_shared<Node>(u_x,u_y,u_z);
            nodes.emplace_back(nd);

        }
        return nodes;


    }

    int Analysis::bc_size()const{
        int num_bcs =0;
        for(auto& bc : bcs){
            num_bcs += bc.nodes.size();
        }
        return num_bcs;
    }

    int Analysis::get_size()const{
        return num_free_dofs;
    }

    int Analysis::total_size()const{
        return get_size() + bc_size();
    }
    
    int Analysis::number_of_elements()const{
        return mesh->number_of_elements();
    }

    IElement_ptr Analysis::get_element(int el_number)const{
        return mesh->get_element(el_number);
    }

    Mesh_ptr Analysis::get_mesh()const{
        return mesh;
    }

    const IDMat& Analysis::get_ID()const{
        return ID;
    }

    const VectorNodes& Analysis::nodes()const{
        return mesh->get_nodes();
    }

    const VectorElements& Analysis::elements()const{
        return mesh->get_elements();
    }

    std::vector<int> Analysis::get_free_dofs()const{

        std::vector<int> free_dofs;
        free_dofs.reserve(num_free_dofs);

        int k=0;
        for(auto& val: ID){
            if(val>=0){
                free_dofs.emplace_back(k);
            }
            k++;
        }
        return free_dofs;
    }

    std::vector<int> Analysis::get_bc_dofs()const{

        std::vector<int> bc_dofs;
        bc_dofs.reserve(bc_size());

        int k=0;
        for(auto& val: ID){
            if(val<0){
                bc_dofs.emplace_back(k);
            }
            k++;
        }
        return bc_dofs;
    }

    void Analysis::clear_forces(){
        fg_data = nullptr;
        forces.clear();
    }

    void Analysis::add_force(DOFType dof, int node_number, double value){
        fg_data = nullptr;
        create_force(forces,dof,node_number,value);
    }

    void Analysis::add_force(DOFType dof, std::vector<int> nodes, double value){
        fg_data = nullptr;
        create_force(forces,dof,nodes,value);
    }

    void Analysis::add_force(DOFType dof, IGeometry_ptr geometry, double value, int integration_points){
        fg_data = nullptr;
        create_force(forces,mesh,dof,geometry,value,integration_points);
    }

    void Analysis::add_force(DOFType dof, IGeometry_ptr geometry, Evalfn func, int integration_points){
        fg_data = nullptr;
        create_force(forces,mesh,dof,geometry,func,integration_points);        
    }

    // ================================================================
    // Class: AnalysisBuilder
    // Creates the AnalysisBuilder class.
    // =================================================================
    
    void AnalysisBuilder::get_degrees_of_freedom(){

        std::unordered_set<DOFType> seen;

        for(auto& el : mesh->get_elements()){
            std::vector<DOFType> dofs = el->dofs();
            for(auto& dof : dofs){
                if (seen.insert(dof).second) {
                    unique_dofs.push_back(dof);
                }
            }
        }
        
    }

    void AnalysisBuilder::create_id_matrix(){

        size_t cols = mesh->number_of_nodes();
        ID = IDMat(unique_dofs,cols);
    }


    void AnalysisBuilder::populate_id_matrix(){

        for(int i=0; i<bc_vector.size(); i++){
            DOFType type = bc_vector[i].type;
            for(auto& node: bc_vector[i].nodes){
                ID(type,node) = -i-1;
            }
        }
        
        num_free_dofs = 0;
        for(auto& id_val:ID){
            if(id_val==0){
                id_val = num_free_dofs++;
            }
        }


    }


    void AnalysisBuilder::add_boundary_condition(DOFType dof, int node_number, double value){
        bc_vector.push_back(BoundaryCondition(dof,node_number,value));
    }

    void AnalysisBuilder::add_boundary_condition(DOFType dof, std::vector<int> nodes, double value){
        for(auto& node_number : nodes){
            bc_vector.push_back(BoundaryCondition(dof,node_number,value));
        }
    }

    void AnalysisBuilder::add_boundary_condition(DOFType dof, IGeometry_ptr geometry, double value){

        std::vector<int> nodes = get_mesh_nodes_in_geometry(mesh, geometry);
        bc_vector.push_back(BoundaryCondition(dof,nodes,value));
    }

    void AnalysisBuilder::add_force(DOFType dof, int node_number, double value){
        create_force(forces,dof,node_number,value);
    }

    void AnalysisBuilder::add_force(DOFType dof, std::vector<int> nodes, double value){
        create_force(forces,dof,nodes,value);
    }

    void AnalysisBuilder::add_force(DOFType dof, IGeometry_ptr geometry, double value, int integration_points){
        create_force(forces,mesh,dof,geometry,value,integration_points);
    }

    void AnalysisBuilder::add_force(DOFType dof, IGeometry_ptr geometry, Evalfn func, int integration_points){
        create_force(forces,mesh,dof,geometry,func,integration_points);        
    }


    Analysis AnalysisBuilder::build(){

        get_degrees_of_freedom();
        create_id_matrix();
        populate_id_matrix();

        return Analysis(
            mesh,
            ID,
            bc_vector,
            forces,
            num_free_dofs);
    }

    #ifdef USE_PETSC

        std::unique_ptr<Mat> create_PETSc_Matrix_from_triplets(const std::vector<Triplet>& triplets, int n){

            std::unique_ptr<Mat> A = std::make_unique<Mat>();

            std::vector<int> nnz_per_row(n,0);
            for(const auto& triplet : triplets){
                if(nnz_per_row[triplet.row()] < n)
                    nnz_per_row[triplet.row()]++;
            }

            MatCreateSeqAIJ(PETSC_COMM_SELF, n, n, 0, nnz_per_row.data(), A.get());


            // Insert sparsity pattern only ONCE
            for (const auto& triplet : triplets) {
                int i = triplet.row();
                int j = triplet.col();
                double value = triplet.value();
                MatSetValues(*A, 1, &i, 1, &j, &value, ADD_VALUES);
            }

            MatAssemblyBegin(*A, MAT_FINAL_ASSEMBLY);
            MatAssemblyEnd(*A, MAT_FINAL_ASSEMBLY);

            return A;
        }

        const Mat& Analysis::get_PETSc_K(){

            // Create PETSc matrix K if it does not exist
            if(!Kg_data){
                Kg_data = std::make_unique<std::vector<Triplet>>(assemble(Assembler::Stiffness));
                if(K) MatDestroy(&(*K));
                K = create_PETSc_Matrix_from_triplets(*Kg_data, num_free_dofs);
            }
            else if(!K){
                K = create_PETSc_Matrix_from_triplets(*Kg_data, num_free_dofs);
            }
            return *K;
        }
        const Mat& Analysis::get_PETSc_M(){

            if(!Mg_data){
                Mg_data = std::make_unique<std::vector<Triplet>>(assemble(Assembler::Mass));
                if(M) MatDestroy(&(*M));
                M = create_PETSc_Matrix_from_triplets(*Mg_data, num_free_dofs);
            }
            else if(!M){
                M = create_PETSc_Matrix_from_triplets(*Mg_data, num_free_dofs);
            }
            return *M;
        }
        PetscObjects& Analysis::get_PETSc_objects(){

            if(!obj){

                const Mat* Kmat = &get_PETSc_K();
                obj = std::make_unique<PetscObjects>();
                obj->Kmat = Kmat;
                obj->n = num_free_dofs;
                KSPCreate(PETSC_COMM_SELF, &obj->ksp);
                KSPSetOperators(obj->ksp, *obj->Kmat, *obj->Kmat);  // A as matrix and preconditioner
                KSPSetType(obj->ksp, KSPPREONLY);
                KSPGetPC(obj->ksp, &obj->pc);
                PCSetType(obj->pc, PCLU);
                KSPSetUp(obj->ksp);

            }
            return *obj;
        }
    #endif

} // namespace finelc
