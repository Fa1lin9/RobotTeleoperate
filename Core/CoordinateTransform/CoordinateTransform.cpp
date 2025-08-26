#include <CoordinateTransform/CoordinateTransform.hpp>
#include <CoordinateTransform/VisionPro2CrpRobotTrans.hpp>

CoordinateTransform::CoordinateTransform(){

}

CoordinateTransform::~CoordinateTransform(){

}

boost::shared_ptr<CoordinateTransform> CoordinateTransform::GetPtr(const CoordinateTransform::config &config_){
    switch (config_.type) {
        case TransType::VisionPro2CrpRobot :{
           return boost::make_shared<VisionPro2CrpRobotTrans>(config_);
        }
        default:{
            return nullptr;
        }
    }
}
