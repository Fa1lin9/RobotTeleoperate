#include <RobotTeleoperate/RobotTeleoperate.hpp>
#include <Ti5RobotTeleoperate/Ti5RobotTeleoperate.hpp>

RobotTeleoperate::RobotTeleoperate(){

}

RobotTeleoperate::~RobotTeleoperate(){

}

boost::shared_ptr<RobotTeleoperate> RobotTeleoperate::GetPtr(const RobotTeleoperate::BasicConfig &config_){
    switch (config_.type) {
        case RobotTeleoperate::Type::Ti5Robot :{
           return boost::make_shared<Ti5RobotTeleoperate>(config_);
        }
        default:{
            return nullptr;
        }
    }
}

