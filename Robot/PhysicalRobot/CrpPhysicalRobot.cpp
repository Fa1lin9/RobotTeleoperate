#include "CrpPhysicalRobot.hpp"

CrpPhysicalRobot::CrpPhysicalRobot(const PhysicalRobot::config &config_){

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
//    ti5_socket_server(0,0,SERVER_PORT);

    if(startFlag){
          std::cout << GREEN
                    << " Successfully initialized CAN ! "
                    << RESET << std::endl;

          this->connectFlag = true ;
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
    return this->connectFlag;
}

bool CrpPhysicalRobot::isConnect(){
    if(this->connectFlag){
        std::cout<< RED << typeid(*this).name() << " is connected ! " << RESET << std::endl;
    } else {
        std::cout<< RED << typeid(*this).name() << " is not connected ! " << RESET << std::endl;
    }
    return this->isConnect();
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

    // TODO
}

std::vector<double> CrpPhysicalRobot::GetJointsAngle(){

}
