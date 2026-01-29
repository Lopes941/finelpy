#pragma once

namespace finelc
{
    
    /**
     * @struct IteratorParameters
     * 
     * @brief Structure to hold parameters for node iteration.
     * 
     * This structure encapsulates the parameters needed to define a range of nodes
     * for iteration, including the step size, total size, and starting index.
     */
    struct IteratorParameters{
        int step;
        int size;
        int start;

        IteratorParameters(int size_=0, int start_=0, int step_=1): size(size_), step(step_), start(start_) {}

        IteratorParameters(const IteratorParameters& other){
            step = other.step;
            size = other.size;
            start = other.start;
        }

        bool operator==(const IteratorParameters& other) const{
            return step==other.step &&
                size==other.size &&
                start==other.start;
        }

         bool operator!=(const IteratorParameters& other)const{
            return !(*this == other);
        }
    };

    /**
     * @class NodeRangeIterator
     * 
     * @brief Iterator class for iterating over a range of node indices.
     * 
     * This class provides an iterator to traverse a range of node indices based on
     * specified parameters such as step size, total size, and starting index.
     */
    class NodeRangeIterator{

        private:
            int current_index=0;
            IteratorParameters parameters;

            bool done=false;


        public:

            NodeRangeIterator(): done(true){}

            NodeRangeIterator(IteratorParameters params): parameters(params) {}

            ~NodeRangeIterator()=default;

            NodeRangeIterator& operator++(){
                ++current_index;
                if(current_index==parameters.size){
                    done = true;
                }
                return *this;
            }

            bool operator==(const NodeRangeIterator& other) const{
                if(done == other.done) return true;
                return parameters==other.parameters &&
                    current_index==other.current_index;
            }

            bool operator!=(const NodeRangeIterator& other)const{
                return !(*this == other);
            }

            int value()const {return parameters.start + parameters.step*current_index;}


    };

    
} // namespace finelc
