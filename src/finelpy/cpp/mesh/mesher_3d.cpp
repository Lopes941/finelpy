
#include <finelc/enumerations.h>

#include <finelc/mesh/mesh.h>
#include <finelc/mesh/meshers.h>

#include <finelc/mesh/node_iterators.h>

#include <finelc/elements/element.h>



#include <exception>
#include <iostream>

namespace finelc{


    MeshBuilderHexahedron::MeshBuilderHexahedron(
        IGeometry_ptr domain_, 
        IElement_ptr el_): 
        MeshBuilder(el_,domain_){

        if(!domain){
            throw std::runtime_error("Null domain passed to MeshBuilderHexahedron");
        }

        if(domain->name() != GeometryType::HEXAHEDRON){
            throw std::runtime_error("Incorrect geometry type. This mesher only accepts hexahedron meshes.");
        }

        Hexahedron_ptr hex = std::dynamic_pointer_cast<Hexahedron>(domain);

        if(!hex){
            throw std::runtime_error("Domain is not a Hexahedron instance");
        }

        if(element->get_shape() != ShapeType::HEX8){
            throw std::runtime_error("Hexahedron Mesh builder only accepts QUAD4 and TRI3 elements.");
        }

        const Point dimensions = hex->get_dimension();
        Lx = dimensions.x;
        Ly = dimensions.y;
        Lz = dimensions.z;

    }

    void MeshBuilderHexahedron::create_hex(){

        int num_nodes;
        int ndx, ndy, ndz;
        double dx, dy, dz;
        if(element->get_shape() == ShapeType::HEX8){
            dx = lx;
            dy = ly;
            dz = lz;
            ndx = nx+1;
            ndy = ny+1;
            ndz = nz+1;
            num_nodes = ndx*ndy*ndz;
        }
        
        VectorNodes nodes;
        nodes.reserve(num_nodes);

        Hexahedron_ptr hex = std::dynamic_pointer_cast<Hexahedron>(domain);

        // Calculating nodes
        Point origin = hex->get_origin();
        for(int z=0; z<ndz; z++){
            for(int y=0; y<ndy; y++){
                for(int x=0; x<ndx; x++){
                    Node_ptr node = std::make_shared<Node>(   
                                dx * x + origin.x,
                                dy * y + origin.y,
                                dz * z + origin.z);
                    nodes.emplace_back(node);
                }
            }
        }
        mesh->set_nodes(std::move(nodes));

        // Calculating nodes from geometry objects
        NodeIterator_ptr iterator = std::make_shared<NodeRangeIterator>(num_nodes,0);
        domain->set_iterator(iterator);


        std::shared_ptr<std::vector<IArea>> faces = hex->get_faces();

        iterator = std::make_shared<NodeRangeIterator>(ndy*ndx,0);
        (*faces)[0].set_iterator(iterator);

        iterator = std::make_shared<NodeStrideRangeIterator>(ndx,0,1,ndy*ndx,ndz);
        (*faces)[1].set_iterator(iterator);

        iterator = std::make_shared<NodeStrideRangeIterator>(ndy,nx,ndx,ndy*ndx,ndz);
        (*faces)[2].set_iterator(iterator);

        iterator = std::make_shared<NodeStrideRangeIterator>(ndx,ny*ndx,1,ndy*ndx,ndz);
        (*faces)[3].set_iterator(iterator);

        iterator = std::make_shared<NodeStrideRangeIterator>(ndy,0,ndx,ndy*ndx,ndz);
        (*faces)[4].set_iterator(iterator);

        iterator = std::make_shared<NodeRangeIterator>(ndy*ndx,ndx*ndy*nz);
        (*faces)[5].set_iterator(iterator);



        // Calculating elements
        if(element->get_shape() == ShapeType::HEX8){
            populate_hex8();
        }
        
    }

    void MeshBuilderHexahedron::populate_hex8(){

        VectorElements elements;
        int num_elements = nx*ny*nz;
        elements.reserve(num_elements);
        std::vector<int> elem_nodes(8);
        for(int z=0; z<nz; z++){
            int offset_z = z*(nx+1)*(ny+1);
            for(int y=0; y<ny; y++){
                int offset = y*(nx+1) + offset_z;
                for(int x=0; x<nx; x++){
                    elem_nodes[0] = x + offset;
                    elem_nodes[1] = elem_nodes[0]+1;
                    elem_nodes[3] = elem_nodes[0] + (nx+1);
                    elem_nodes[2] = elem_nodes[3]+1;

                    elem_nodes[4] = elem_nodes[0] + (nx+1)*(ny+1);
                    elem_nodes[5] = elem_nodes[4]+1;
                    elem_nodes[7] = elem_nodes[4] + (nx+1);
                    elem_nodes[6] = elem_nodes[7]+1;

                    element->set_node_numbering(elem_nodes, mesh->get_nodes());
                    elements.emplace_back(element->copy(true));
                }
            }
        }
        mesh->set_elements(elements);
    }


    void MeshBuilderHexahedron::set_size_from_grid(){
        lx = Lx/nx;
        ly = Ly/ny;
        lz = Lz/nz;
    }

    void MeshBuilderHexahedron::set_element_size(double lx_, double ly_, double lz_){
            lx = lx_<Lx ? lx_ : Lx;
            ly = ly_<Ly ? ly_ : Ly;
            lz = lz_<Lz ? lz_ : Lz;

            nx = Lx/lx;
            ny = Ly/ly;
            nz = Lz/lz;
            set_size_from_grid();
    }

    void MeshBuilderHexahedron::set_grid(int nx_, int ny_, int nz_){
        nx = nx_;
        ny = ny_;
        nz = nz_;

        set_size_from_grid();
    }

    Mesh_ptr MeshBuilderHexahedron::build(){
        create_hex();
        return mesh;
    }

    
} // namespace finelc
