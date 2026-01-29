#pragma once

#include <finelc/matrix.h>

#include <finelc/analysis/analysis.h>
#include <finelc/result/result.h>

#include <memory>

namespace finelc{

    class StaticSolver{

        private:

            Analysis_ptr analysis;
            std::unique_ptr<SolverType> type;
            std::unique_ptr<Solver> solver=nullptr;

            void default_solver();

        public:
        
            StaticSolver(Analysis_ptr anal): 
                analysis(anal) {}
            StaticSolver(Analysis_ptr anal, SolverType type_): 
                analysis(anal), type(std::make_unique<SolverType>(type_)) {}
            ~StaticSolver()=default;

            StaticResult solve();
    };

    
    
} // namespace finelc
