#include "CrpPhysicalRobot.hpp"

CrpPhysicalRobot::CrpPhysicalRobot(const PhysicalRobot::config &config_){
    l_solver.gap=0.5;
    r_solver.gap=0.5;
    l_controller.max_vel=0.6;
    r_controller.max_vel=0.6;

}

CrpPhysicalRobot::~CrpPhysicalRobot(){

}

bool CrpPhysicalRobot::Connect(){
    // 如果是在成功连接之后调用的话
    // 就直接通过Flag来判断
    // 避免再次重复连接的操作
    if(this->connectFlag){
        std::cout<< GREEN
                 << typeid(*this).name()
                 << " is already connected ! "
                 << RESET << std::endl;
    }

    // Start to connect
    std::vector<string> productSerialNumbers = query_can();
    if (productSerialNumbers.empty())
    {
        std::cout << RED
                  << "Can't find any USB device, plz retry！"
                  << RESET << std::endl;
        return false;
    }
    else
    {
        std::cout << RED
                  << " The CAN device is Founded ! "
                  << RESET << std::endl;
        std::cout << CYAN
                  << "CAN device serial number："
                  << RESET;
        for (const string &serialNumber : productSerialNumbers)
        {
            std::cout << CYAN
                      << serialNumber
                      << RESET << std::endl;
        }
    }
    std::string ip = ip_address();
    cout << MAGENTA << " IP = " << ip << RESET << endl;

    bool startFlag = Start();
//    bool startFlag = CanDriver::InitCan();
//    ti5_socket_server(0,0,SERVER_PORT);

    if(startFlag){
        std::cout << GREEN
                    << " Successfully initialize the CAN ! "
                    << RESET << std::endl;

          this->connectFlag = true ;
    }else{
        std::cout << RED
                  << " Can't initialize the CAN ! "
                  << RESET << std::endl;
    }
    return this->connectFlag;
}

bool CrpPhysicalRobot::Disconnect(){
    bool exitFlag = Exit();
    if(exitFlag){
          std::cout << RED
                    << " Successfully exited CAN ! "
                    << RESET << std:: endl;
          this->connectFlag = false;
    }
//    CanDriver::CloseCan();
    return this->connectFlag;
}

bool CrpPhysicalRobot::isConnect(){
    if(this->connectFlag){
        std::cout<< RED << typeid(*this).name() << " is connected ! " << RESET << std::endl;
    } else {
        std::cout<< RED << typeid(*this).name() << " is not connected ! " << RESET << std::endl;
    }
    return this->connectFlag;
}

bool CrpPhysicalRobot::EmergencyStop(){
    brake(LEFT_ARM,0,0);
    brake(RIGHT_ARM,1,0);
    std:cout << RED
             << " Emergency Stop !!! "
             << RESET << std::endl;
    return true;
}

bool CrpPhysicalRobot::BackToZero(){
    if(!this->isConnect()){ return false; }

    mechanical_arm_origin(ArmSide::LEFT_ARM,0,0);

    return true;
}

std::vector<double> CrpPhysicalRobot::GetJointsAngle(){
    LOG_FUNCTION;
    if(!this->isConnect()){ return {}; }

    std::vector<double> jointsAngle;
    float currentAngle[this->dofArm];

    // For Left Arm
    get_current_angle(ArmSide::LEFT_ARM, currentAngle, 0, 0);
    for(size_t i = 0; i < this->dofArm; i++){
        jointsAngle.push_back(currentAngle[i]);
        std::cout << " Left Arm Joint"
                  << i << " : "
                  << fixed << setprecision(4)
                  << currentAngle[i] << std::endl;
    }

    // For Right Arm
    get_current_angle(ArmSide::RIGHT_ARM, currentAngle, 0, 0);
    for(size_t i = 0; i < this->dofArm; i++){
        jointsAngle.push_back(currentAngle[i]);
        std::cout << " Right Arm Joint"
                  << i << " : "
                  << fixed << setprecision(4)
                  << currentAngle[i] << std::endl;
    }

    return {};
}

void CrpPhysicalRobot::Info(){
    LOG_FUNCTION;
    ArmSide armSide;

    std::cout<< RED
             << " ------------------------------------------ "
             << RESET << std::endl;

    std::cout<< " Left Arm ID: " << Left_arm_id<<std::endl;
    std::cout<< " Right Arm ID: " << Right_arm_id<<std::endl;

    // for head
    {
        std::cout<< GREEN
                 << " Head's canDevice and canIndex : "
                 << RESET << std::endl;
        for(size_t i = 0;i < this->dofHead ;i++){
            std::cout << GREEN
                      << " canDevice: " << size_t(0)
                      << " canIndex " << this->headCanID[i]
                      << RESET << std::endl;
        }
    }

    std::cout<< RED
             << " ------------------------------------------ "
             << RESET << std::endl;

    // for Left Arm
    {
        std::cout<< YELLOW
                 << " Left arm's canDevice and canIndex : "
                 << RESET << std::endl;
        for(size_t i = 0;i < this->dofArm ;i++){
            std::cout << YELLOW
                      << " canDevice: " << size_t(0)
                      << " canIndex " << this->leftArmCanID[i]
                      << RESET << std::endl;
        }
    }

    std::cout<< RED
             << " ------------------------------------------ "
             << RESET << std::endl;

    // for Right Arm
    {
        std::cout<< BLUE
                 << " Right arm's canDevice and canIndex : "
                 << RESET << std::endl;
        for(size_t i = 0;i < this->dofArm ;i++){
            std::cout << BLUE
                      << " canDevice: " << size_t(0)
                      << " canIndex " << this->rightArmCanID[i]
                      << RESET << std::endl;
        }
    }

    std::cout<< RED
             << " ------------------------------------------ "
             << RESET << std::endl;

}

void CrpPhysicalRobot::GetJointsStatus(){
    if(!this->isConnect()){ return; }
    int32_t dataList[this->dofArm];

    get_mechanicalarm_status(ArmSide::LEFT_ARM,0,0,dataList);

    for(size_t i = 0;i < this->dofArm;i++){
        std::cout<<" Left Arm Joint " <<i <<" Status: "<<dataList[i]<<std::endl;
    }

    get_mechanicalarm_status(ArmSide::RIGHT_ARM,1,0,dataList);

    for(size_t i = 0;i < this->dofArm;i++){
        std::cout<<" Right Arm Joint " <<i <<" Status: "<<dataList[i]<<std::endl;
    }
}

bool CrpPhysicalRobot::MoveJ(const std::vector<double> &jointsAngle_){
//    std::cout << RED
//              << " Sorry,please use MoveJ(const PhysicalRobot::CrpRobotConfig& config_) to control the CrpRobot ! "
//              << RESET
//              << std::endl;
    if(!this->isConnect()){ return false; }
    float jointsAngle[this->dofArm];

    for(size_t i=0;i<this->dofArm;i++){
        jointsAngle[i]=jointsAngle_[i];
    }

    joint_to_move(ArmSide::LEFT_ARM,jointsAngle,0,0);

    return true;
}

bool CrpPhysicalRobot::MoveL(){

    return true;
}

bool CrpPhysicalRobot::MoveJ(const PhysicalRobot::CrpRobotConfig &config_){
    if(!this->isConnect()){ return false; }

    // for left arm
    if(config_.useLeftArm){
        this->SendRecvJoints(config_.leftArmJointsValue,
                             this->dofArm,
                             this->leftArmCanDevice,
                             this->leftArmCanID,
                             "Left Arm"
                             );
    }

    // for right arm
    if(config_.useRightArm){
        this->SendRecvJoints(config_.rightArmJointsValue,
                             this->dofArm,
                             this->rightArmCanDevice,
                             this->rightArmCanID,
                             "Right Arm"
                             );
    }

    // for head
    if(config_.useHead){
        this->SendRecvJoints(config_.headJointsValue,
                             this->dofHead,
                             this->headCanDevice,
                             this->headCanID,
                             "Head"
                             );
    }

    // for waist
    if(config_.useWaist){
        this->SendRecvJoints(config_.waistJointsValue,
                             this->dofWaist,
                             this->waistCanDevice,
                             this->waistCanID,
                             "Waist"
                             );
    }

    // for hand
    // TODO

    return true;
}

bool CrpPhysicalRobot::SendRecvJoints(const std::vector<double>& jointsValue,
                    size_t dof,
                    size_t canDevice,
                    const std::vector<size_t>& canID,
                    const std::string& partName) {
    if (jointsValue.empty()) {
        throw std::invalid_argument("[" + partName + "] The joints vector is empty!");
        return false;
    } else if (jointsValue.size() != dof) {
        throw std::length_error("[" + partName + "] The joints vector size does not match!");
        return false;
    }

    for (size_t i = 0; i < dof; i++) {
        double value = jointsValue[i];
        auto result = CanDriver::SendRecvPosition(canDevice, canID[i], &value);
        if (result.has_value()) {
            std::cout << "[" << partName << "] "
                      << "Can ID: " << std::fixed << std::setprecision(4) << std::get<0>(result.value())
                      << " Value: " << std::fixed << std::setprecision(4) << std::get<1>(result.value())
                      << std::endl;
        } else {
            std::cerr << "[" << partName << "] "
                      << "Failed to send position for Can ID: " << canID[i]
                      << std::endl;
        }
    }
    return true;
}
