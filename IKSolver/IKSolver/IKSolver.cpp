#include <IKSolver/IKSolver.hpp>
#include <CrpRobotIKSolver/CrpRobotIKSolver.hpp>

IKSolver::IKSolver(){

}

IKSolver::~IKSolver(){

}

boost::shared_ptr<IKSolver> IKSolver::GetPtr(const IKSolver::BasicConfig &config_){
    switch (config_.type) {
        case IKSolver::Type::CrpRobot :{
           return boost::make_shared<CrpRobotIKSolver>(config_);
        }
        default:{
            return nullptr;
        }
    }
}
