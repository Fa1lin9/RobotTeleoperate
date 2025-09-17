#include <iostream>
#include <CoordinateTransform/CoordinateTransform.hpp>
#include <DataCollector/VisionProCollector.hpp>
#include <thread>

int main(){
    std::string address = "tcp://127.0.0.1:5555";
    auto visionProCollector = VisionProCollector(address);

    // CoordinateTransform
    Eigen::Matrix4d temp;
    temp<<1,0,0,0,
         0,-1,0,0,
         0,0,-1,0,
         0,0,0,1;
    CoordinateTransform::BasicConfig basicConfig;
    basicConfig.type = TransformType::VisionPro2CrpRobot;
    basicConfig.T_Head2Waist = Eigen::Matrix4d::Identity();
    basicConfig.T_XR2Robot <<  -1, 0, 0, 0,
                                0, 1, 0, 0,
                                0, 0, -1, 0,
                                0, 0, 0, 1;
    basicConfig.T_Robot2LeftWrist <<0.0, 1.0, 0.0, 0.0,
                                    0.0, 0.0,-1.0, 0.0,
                                   -1.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 1.0;
    basicConfig.T_Robot2LeftWrist = temp * basicConfig.T_Robot2LeftWrist;
    basicConfig.T_Robot2RightWrist <<   0.0,-1.0, 0.0, 0.0,
                                        0.0, 0.0, 1.0, 0.0,
                                       -1.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 1.0;
    auto transformPtr = CoordinateTransform::GetPtr(basicConfig);

    while(1){
        std::vector<Eigen::Matrix4d> msg = visionProCollector.GetValue();

        if(msg.size()==0 || msg.empty()){
            std::string error = " The message received is not qualified !";
            throw std::invalid_argument(error);
            continue;
        }

        if(1){
            std::cout << "--------------------------" << std::endl;
            std::cout << "Head Pose:\n" << msg[0] << std::endl;
            std::cout << "Left Wrist Pose:\n" << msg[1] << std::endl;
            std::cout << "Right Wrist Pose:\n" << msg[2] << std::endl;
            std::cout << "--------------------------" << std::endl;
        }

        CoordinateTransform::MsgConfig msgConfig{
            .head2xrWorldPose = msg[0],
            .leftWrist2xrWorldPose = msg[1],
            .rightWrist2xrWorldPose = msg[2],
        };

        std::vector<Eigen::Matrix4d> transformedMsg = transformPtr->Transform(msgConfig);

        if(1){
            std::cout << "--------------------------" << std::endl;
            std::cout << "Transformed Left Wrist Pose:\n" << transformedMsg[0] << std::endl;
            std::cout << "Transformed Right Wrist Pose:\n" << transformedMsg[1] << std::endl;
            std::cout << "--------------------------" << std::endl;
        }
    }

    return 0;

}
