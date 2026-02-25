#pragma once

#include <memory>

namespace finelc
{

    /**
     * @class NodeIterator
     * 
     * @brief Abstract Iterator class for iterating over a range of node indices.
     * 
     * This class provides an interface to traverse a range of node indices based on
     * specified parameters such as step size, total size, and starting index.
     */
    class NodeIterator{

        protected:

            int size;
            int current_index =0;
            
            NodeIterator(int size_): done(false), size(size_){}
            
        public:

            NodeIterator(): done(true) {}
            virtual ~NodeIterator()=default;

            bool done = false;

            int get_size()const{return size;}

            void reset() {
                current_index=0;
                done = (size == 0);    
            }

            virtual int value() const{return -1;}

            NodeIterator& operator++(){
                if (!done) {
                    ++current_index;
                    if (current_index >= size) {
                        done = true;
                    }
                }
                return *this;
            }
    };

    
    using NodeIterator_ptr = std::shared_ptr<NodeIterator>;
    
    /**
     * @class NodeRangeIterator
     * 
     * @brief Iterator class for iterating over a range of node indices.
     * 
     * This class provides an iterator to traverse a range of node indices based on
     * specified parameters such as step size, total size, and starting index.
     */
    class NodeRangeIterator: public NodeIterator{

        private:
            int step;
            int start;

        public:

            NodeRangeIterator(int size_=0, int start_=0, int step_=1):
                NodeIterator(size_),
                step(step_),
                start(start_)
                {}

            ~NodeRangeIterator()=default;

            int value()const final{return start + step * current_index;}

            

    };

    class NodeStrideRangeIterator: public NodeIterator{

        private:
            int step;
            int start;
            int stride;
            int num_strides;
            int range_size;

        public:

            NodeStrideRangeIterator(int range_size_=0, 
                int start_=0, 
                int step_=1,
                int stride_=0,
                int num_strides_=0):
                NodeIterator(range_size_*num_strides_),
                range_size(range_size_),
                step(step_),
                start(start_),
                stride(stride_),
                num_strides(num_strides_)
                {}

            ~NodeStrideRangeIterator()=default;
            

            int value()const final{
                return start + (current_index%range_size)*step + (current_index/range_size)*stride ;
            }

            

    };



    
    
} // namespace finelc
