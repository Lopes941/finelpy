
#include <finelc/mesh/mesh.h>
#include <finelc/mesh/meshers.h>


#include <algorithm>
#include <numeric>

namespace finelc{


    void Mesh::add_element(IElement_ptr new_el){
        elements.emplace_back(new_el);
    }

    void Mesh::add_elements(const VectorElements& new_els){
        elements.reserve(elements.size() + new_els.size());
        for(const auto& new_el: new_els){
            add_element(new_el);
        }
    }

    void Mesh::set_elements(const VectorElements& els){
        elements = els;
    }

    void Mesh::add_node(Node_ptr new_node){
        nodes.emplace_back(new_node);
    }

    void Mesh::add_nodes(const VectorNodes& new_nodes){
        nodes.reserve(nodes.size() + new_nodes.size());
        for(const auto& new_node: new_nodes){
            add_node(new_node);
        }
    }

    void Mesh::set_nodes(const VectorNodes& nds){
        nodes = nds;
    }

    IElement_ptr Mesh::get_element(int el)const{
        return elements.at(el);
    }    

    Node_ptr Mesh::get_node(int nd)const{
        return nodes.at(nd);
    }    

    int Mesh::find_node(const Point& p)const{
        auto it = std::min_element(nodes.begin(), nodes.end(),
        [&](const Point_ptr a, const Point_ptr b){
            return dist(*a,p) < dist(*b,p);
        });

        return static_cast<int>(std::distance(nodes.begin(), it));
    }

    const VectorNodes& Mesh::get_nodes() const{
        return nodes;
    }

    const VectorElements& Mesh::get_elements() const{
        return elements;
    }

    const VectorNodes Mesh::element_center()const{
        VectorNodes centers;
        centers.reserve(elements.size());
        for(auto& el :elements){
            centers.emplace_back(std::make_shared<Point>(el->geometric_center()));
        }
        return centers;
    }

    size_t Mesh::number_of_nodes() const{
        return nodes.size();
    }

    size_t Mesh::number_of_elements() const{
        return elements.size();
    }

    int Mesh::find_element(const Point& loc) const{

        VectorNodes element_centers = element_center();
        std::vector<int> inds(number_of_elements());
        std::iota(inds.begin(),inds.end(),0);
        std::sort(inds.begin(), inds.end(),
              [&element_centers, &loc](int i1, int i2) {return dist(*element_centers[i1],loc) < dist(*element_centers[i2],loc);});

        int index = -1;
        for(auto& ind: inds){
            VectorNodes nodes = elements[ind]->get_nodes();

            IGeometry_ptr geo = elements[ind]->geometry();
            if(geo->is_inside(loc)){
                index = ind;
                break;
            }
        }
        if(index==-1){
            throw std::runtime_error("No element contains point");
        }
        return index;
    }


} // namespace finelc
