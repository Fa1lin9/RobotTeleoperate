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

const std::unordered_map<std::string, CoordinateTransform::Type> CoordinateTransform::typeMap = {
    {"VisionPro2Ti5Robot", CoordinateTransform::Type::VisionPro2Ti5Robot}
};

CoordinateTransform::Type CoordinateTransform::GetTypeFromStr(const std::string& str){
    auto temp = CoordinateTransform::typeMap.find(str);
    if(temp != CoordinateTransform::typeMap.end()){
        return temp->second;
    }

    throw std::invalid_argument("[CoordinateTransform::GetTypeFromStr] Invalid string");
}
