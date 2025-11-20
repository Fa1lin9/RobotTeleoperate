#include <PhysicalRobot/PhysicalRobot.hpp>
#include <Ti5PhysicalRobot/Ti5PhysicalRobot.hpp>

PhysicalRobot::PhysicalRobot(){

}

PhysicalRobot::~PhysicalRobot(){

}

bool PhysicalRobot::BackToInitPose(const PhysicalRobot::Ti5RobotConfig& config_){
    LOG_FUNCTION;
    return true;
}

bool PhysicalRobot::BackToZero(const PhysicalRobot::Ti5RobotConfig& config_){
    LOG_FUNCTION;
    return true;
}

bool PhysicalRobot::MoveJ(const PhysicalRobot::Ti5RobotConfig& config_){
    LOG_FUNCTION;
    return true;
}

boost::shared_ptr<PhysicalRobot> PhysicalRobot::GetPtr(const PhysicalRobot::BasicConfig &config_){
    switch (config_.type) {
        case PhysicalRobot::Type::Ti5Robot :{
           return boost::make_shared<Ti5PhysicalRobot>(config_);
        }
        default:{
            return nullptr;
        }
    }
}

const std::unordered_map<std::string, PhysicalRobot::Type> PhysicalRobot::typeMap = {
    {"Ti5Robot", PhysicalRobot::Type::Ti5Robot}
};

PhysicalRobot::Type PhysicalRobot::GetTypeFromStr(const std::string& str){
    auto temp = PhysicalRobot::typeMap.find(str);
    if(temp != PhysicalRobot::typeMap.end()){
        return temp->second;
    }

    throw std::invalid_argument("[PhysicalRobot::GetTypeFromStr] Invalid string");
}
