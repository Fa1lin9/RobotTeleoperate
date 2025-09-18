#include <PhysicalRobot/PhysicalRobot.hpp>
#include <CrpPhysicalRobot/CrpPhysicalRobot.hpp>

PhysicalRobot::PhysicalRobot(){

}

PhysicalRobot::~PhysicalRobot(){

}

bool PhysicalRobot::BackToInitPose(const PhysicalRobot::CrpRobotConfig& config_){
    LOG_FUNCTION;
    return true;
}

bool PhysicalRobot::BackToZero(const PhysicalRobot::CrpRobotConfig& config_){
    LOG_FUNCTION;
    return true;
}

bool PhysicalRobot::MoveJ(const PhysicalRobot::CrpRobotConfig& config_){
    LOG_FUNCTION;
    return true;
}

boost::shared_ptr<PhysicalRobot> PhysicalRobot::GetPtr(const PhysicalRobot::config &config_){
    switch (config_.type) {
        case PhysicalRobot::Type::CrpRobot :{
           return boost::make_shared<CrpPhysicalRobot>(config_);
        }
        default:{
            return nullptr;
        }
    }
}
