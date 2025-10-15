#include <iostream>

#include <PhysicalRobot/PhysicalRobot.hpp>
#include <boost/shared_ptr.hpp>
#include <math.h>

int main(){

    // Set configeration
    PhysicalRobot::BasicConfig config = {
        .type = PhysicalRobot::Type::CrpRobot,
    };

    boost::shared_ptr<PhysicalRobot> physicalRobotPtr
            = PhysicalRobot::GetPtr(config);

    physicalRobotPtr->Connect();

//    sleep(1);
    std::cout<<"start to back to zero"<<std::endl;

//    physicalRobotPtr->BackToZero();

//    physicalRobotPtr->Info();

//    physicalRobotPtr->isConnect();
//    sleep(3);
//    std::cout<<" start to move !!! "<<std::endl;
    // closely
    // joint1: 0.03 -- 45
    // joint2: 0.03 -- 45
    // joint3: 0.06 -- 90
    // joint4: 0.03 -- 45
    // joint5:
    // joint6: 0.02 -- 45
    // joint7: 0.03 -- 45

//    std::vector<double> jointsAngle_ = { 0.01 , 0.02 , 0.02 , 0.01 , 0 , 0.02, 0.02};
//    physicalRobotPtr->MoveJ(jointsAngle_);

    PhysicalRobot::CrpRobotConfig crpRobotConfig = {
        .useLeftArm = true,
//        .useRightArm = true,
//        .leftArmJointsValue = std::vector<double>{ 0 , -0.3 , 0.4 , 0.3 , 0 , 0.3 , 0.4 },
        .leftArmJointsValue = std::vector<double>{ 0 , 0 , 0 , 0 , 0 , 0 , 0 },
        .rightArmJointsValue = std::vector<double>{ 0 , 0 , 0 , 0 , 0 , 0 , 0 },
    };

//    physicalRobotPtr->MoveJ(crpRobotConfig);

//    sleep(3);

    std::cout<<"q: "<<physicalRobotPtr->GetJointsAngleEigen()<<std::endl;

//    physicalRobotPtr->GetJointsStatus();

    physicalRobotPtr->Disconnect();
}
