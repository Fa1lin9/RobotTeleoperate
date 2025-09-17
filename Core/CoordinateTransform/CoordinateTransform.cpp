#include <CoordinateTransform/CoordinateTransform.hpp>
#include "VisionPro2CrpRobotTransform.hpp"

CoordinateTransform::CoordinateTransform(){

}

CoordinateTransform::~CoordinateTransform(){

}

boost::shared_ptr<CoordinateTransform> CoordinateTransform::GetPtr(const CoordinateTransform::BasicConfig &config_){
    switch (config_.type) {
        case TransformType::VisionPro2CrpRobot :{
           return boost::make_shared<VisionPro2CrpRobotTransform>(config_);
        }
        default:{
            return nullptr;
        }
    }
}
