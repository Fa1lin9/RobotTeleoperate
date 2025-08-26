#include <IKSolver/IKSolver.hpp>
#include <IKSolver/OptimIKSolver.hpp>

IKSolver::IKSolver(){

}

IKSolver::~IKSolver(){

}

boost::shared_ptr<IKSolver> IKSolver::GetPtr(const IKSolver::config &config_){
    switch (config_.type) {
        case SolverType::Optimization :{
           return boost::make_shared<OptimIKSolver>(config_);
        }
        default:{
            return nullptr;
        }
    }
}
