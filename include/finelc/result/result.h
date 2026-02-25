#pragma once

#include <finelc/matrix.h>
#include <finelc/enumerations.h>

#include <finelc/mesh/mesh.h>
#include <finelc/analysis/analysis.h>

#include <memory>

namespace finelc{

    using GridData = std::vector<std::pair<Point,double>>;
    using SupportFn = bool(IElement::*)()const;
    using EvalFn = double(const Vector&, const Vector&, IElement_ptr);
    using EvalFnPtr = EvalFn*;

    extern Vector compute_mean(const Vector& U, Analysis_ptr analysis, ResultData id, int gauss_pts);
    extern GridData compute_grid(const Vector& U, Analysis_ptr analysis, ResultData id, int internal_pts);
    extern SupportFn get_support_func(ResultData id);
    extern EvalFnPtr get_eval_func(ResultData id);
    extern std::vector<Point> create_grid(int internal_pts, int num_dimensions, IntegrationGeometry geom);

    struct MaxMinResult{
        Point pt;
        double val;
    };

    class StaticResult{

        private:
            Analysis_ptr analysis;
            Vector U;

        public:

            StaticResult(Vector U_, Analysis_ptr analysis_): U(U_), analysis(analysis_) {}
            ~StaticResult()=default;

            Vector u()const{
                return analysis->reconstruct_ug(U);
            }

            std::vector<Point> get_points(int internal_pts=10)const;

            #ifdef USE_VTK
                void plot_2D_grid(ResultData id, int internal_pts = 10,bool show_edges = false, bool show_nodes = false)const;
            #endif

            GridData grid_data(ResultData id, int internal_pts = 10)const{
                return compute_grid(U,analysis,id,internal_pts);
            }

            
            MaxMinResult get_max(ResultData id, int internal_pts = 10)const{
                GridData grid = grid_data(id,internal_pts);
                double max_val = grid[0].second;
                Point max_pt = grid[0].first;
                for(auto& data:grid){
                    if(data.second > max_val){
                        max_pt = data.first;
                        max_val = data.second;
                    }
                }
                return {max_pt,max_val};
            }

            MaxMinResult get_min(ResultData id, int internal_pts = 10)const{
                GridData grid = grid_data(id,internal_pts);
                double min_val = grid[0].second;
                Point min_pt = grid[0].first;
                for(auto& data:grid){
                    if(data.second < min_val){
                        min_pt = data.first;
                        min_val = data.second;
                    }
                }
                return {min_pt, min_val};
            }

            double get_value(ResultData id, const Point& loc)const;

            Vector element_mean(ResultData id, int gauss_pts=10)const{
                return compute_mean(U,analysis,id,gauss_pts);
            }

            const VectorNodes& nodes()const{return analysis->nodes();}
            const VectorElements& elements()const{return analysis->elements();}
            Analysis_ptr get_analysis()const{return analysis;}

            double get_compliance()const{
                Vector ug =analysis->reconstruct_ug(U); 
                return analysis->fg().dot(ug);
            }

            Vector compliance_derivative()const{

                int num_elements = analysis->number_of_elements();
                Vector alpha(num_elements);
                Vector interpolation = analysis->get_interpolation_derivative();

                for(int e=0;e<num_elements;e++){

                    IElement_ptr el = analysis->elements()[e];
                    Vector ue = analysis->get_element_ue(U,e);
                    Matrix Ke = el->Ke(ue);

                    alpha(e) = interpolation(e) * ue.dot(Ke * ue);
                }

                return alpha;
            }

    };


    // struct EigenResult{
    //     const Matrix U;
    //     const Vector lambda;
    // };
    
    
} // namespace finelc



