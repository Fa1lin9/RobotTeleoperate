#include <RobotTeleoperate/RobotTeleoperate.hpp>
#include <CrpRobotTeleoperate/CrpRobotTeleoperate.hpp>

RobotTeleoperate::RobotTeleoperate(){

}

RobotTeleoperate::~RobotTeleoperate(){

}

boost::shared_ptr<RobotTeleoperate> RobotTeleoperate::GetPtr(const RobotTeleoperate::BasicConfig &config_){
    switch (config_.type) {
        case RobotTeleoperate::Type::CrpRobot :{
           return boost::make_shared<CrpRobotTeleoperate>(config_);
        }
        default:{
            return nullptr;
        }
    }
}
