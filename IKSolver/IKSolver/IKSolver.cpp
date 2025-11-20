#include <IKSolver/IKSolver.hpp>
#include <Ti5RobotIKSolver/Ti5RobotIKSolver.hpp>

IKSolver::IKSolver(){

}

IKSolver::~IKSolver(){

}

boost::shared_ptr<IKSolver> IKSolver::GetPtr(const IKSolver::BasicConfig &config_){
    switch (config_.type) {
        case IKSolver::Type::Ti5Robot :{
           return boost::make_shared<Ti5RobotIKSolver>(config_);
        }
        default:{
            return nullptr;
        }
    }
}

const std::unordered_map<std::string, IKSolver::Type> IKSolver::typeMap = {
    {"Ti5Robot", IKSolver::Type::Ti5Robot}
};

IKSolver::Type IKSolver::GetTypeFromStr(const std::string& str){
    auto temp = IKSolver::typeMap.find(str);
    if(temp != IKSolver::typeMap.end()){
        return temp->second;
    }

    throw std::invalid_argument("[IKSolver::GetTypeFromStr] Invalid string");
}
