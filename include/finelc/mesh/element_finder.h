#pragma once

#include <finelc/matrix.h>
#include <finelc/enumerations.h>

#include <iostream>
#include <vector>
#include <memory>

namespace finelc{

    class ElementFinder{

        public:

            virtual ~ElementFinder()=default;

            virtual int find_element(const Vector& loc)const=0;

    };

    using ElementFinder_uptr = std::unique_ptr<ElementFinder>;

    class LineElementFinder: public ElementFinder{

        private:
            double lx;

        public:
        
            LineElementFinder()=default;
            LineElementFinder(double lx_):
                lx(lx_)
            {}

            ~LineElementFinder()=default;


            inline int find_element(const Vector& loc)const override{
                return (loc(0)/lx);
            }

    };

    class GridRectangleElementFinder: public ElementFinder{

        private:
            int nx, ny;
            double lx, ly;

            inline int get_column(double locx)const{
                double val = locx/lx;
                if(val == nx){
                    val-=1;
                }
                return val;
            }

            inline int get_row(double locy)const{
                double val = locy/ly;
                if(val == ny){
                    val-=1;
                }
                return val*nx;
            }

        public:
        
            GridRectangleElementFinder()=default;
            GridRectangleElementFinder(double lx_, double ly_, int nx_, int ny_):
                lx(lx_),
                ly(ly_),
                nx(nx_),
                ny(ny_)
            {}

            ~GridRectangleElementFinder()=default;


            inline int find_element(const Vector& loc)const override{
                return get_column(loc(0)) + get_row(loc(1));
            }

    };

} // namespace finelc