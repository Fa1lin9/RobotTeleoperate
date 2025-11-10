#include <CoordinateTransform/CoordinateTransform.hpp>
#include <VisionPro2Ti5RobotTransform/VisionPro2Ti5RobotTransform.hpp>

CoordinateTransform::CoordinateTransform(){

}

CoordinateTransform::~CoordinateTransform(){

}

boost::shared_ptr<CoordinateTransform> CoordinateTransform::GetPtr(const CoordinateTransform::BasicConfig &config_){
    switch (config_.type) {
        case CoordinateTransform::Type::VisionPro2Ti5Robot :{
           return boost::make_shared<VisionPro2Ti5RobotTransform>(config_);
        }
        default:{
            return nullptr;
        }
    }
}
