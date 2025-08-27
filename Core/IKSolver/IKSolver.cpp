#include <IKSolver/IKSolver.hpp>
#include "CrpRobotIKSolver.hpp"

IKSolver::IKSolver(){

}

IKSolver::~IKSolver(){

}

boost::shared_ptr<IKSolver> IKSolver::GetPtr(const IKSolver::config &config_){
    switch (config_.type) {
        case SolverType::CrpRobot :{
           return boost::make_shared<CrpRobotIKSolver>(config_);
        }
        default:{
            return nullptr;
        }
    }
}
