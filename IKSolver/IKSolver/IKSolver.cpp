#include <IKSolver/IKSolver.hpp>
#include <Ti5RobotIKSolver/Ti5RobotIKSolver.hpp>

IKSolver::IKSolver(){

}

IKSolver::~IKSolver(){

}

boost::shared_ptr<IKSolver> IKSolver::GetPtr(const IKSolver::BasicConfig &config_){
    switch (config_.robotType) {
        case RobotType::Ti5Robot :{
           return boost::make_shared<Ti5RobotIKSolver>(config_);
        }
        default:{
            return nullptr;
        }
    }
}



