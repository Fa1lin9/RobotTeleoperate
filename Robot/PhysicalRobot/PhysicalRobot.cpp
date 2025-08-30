#include <PhysicalRobot/PhysicalRobot.hpp>
#include "CrpPhysicalRobot.hpp"

PhysicalRobot::PhysicalRobot(){

}

PhysicalRobot::~PhysicalRobot(){

}

boost::shared_ptr<PhysicalRobot> PhysicalRobot::GetPtr(const PhysicalRobot::config &config_){
    switch (config_.type) {
        case PhysicalRobotType::CrpRobot :{
           return boost::make_shared<CrpPhysicalRobot>(config_);
        }
        default:{
            return nullptr;
        }
    }
}
