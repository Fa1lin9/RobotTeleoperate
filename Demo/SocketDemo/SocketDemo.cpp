#include <zmq.hpp>
#include <iostream>
#include <cstring>
#include <FunctionLogger.hpp>
#include <Core/DataCollector/VisionProCollector.hpp>

struct Matrix4x4 {
    float data[4][4];
};

int main() {
//    LOG_FUNCTION_STATIC;
//    zmq::context_t context(1);
//    zmq::socket_t subscriber(context, zmq::socket_type::sub);

//    subscriber.connect("tcp://127.0.0.1:5555");
//    subscriber.set(zmq::sockopt::subscribe, "");  // 订阅所有消息

//    std::cout << "C++ Subscriber receiving 3x 4x4 matrices..." << std::endl;

//    while (true) {
//        zmq::message_t msg;
//        subscriber.recv(msg, zmq::recv_flags::none);

//        if (msg.size() != sizeof(Matrix4x4)*3) {
//            std::cerr << "Received wrong size: " << msg.size() << std::endl;
//            continue;
//        }

//        Matrix4x4 headPose, leftArmPose, rightArmPose;
//        std::memcpy(&headPose, msg.data(), sizeof(Matrix4x4));
//        std::memcpy(&leftArmPose, (char*)msg.data() + sizeof(Matrix4x4), sizeof(Matrix4x4));
//        std::memcpy(&rightArmPose, (char*)msg.data() + sizeof(Matrix4x4)*2, sizeof(Matrix4x4));

//        std::cout << "Head Pose:" << std::endl;
//        for(int i=0;i<4;i++){
//            for(int j=0;j<4;j++) std::cout << headPose.data[i][j] << " ";
//            std::cout << std::endl;
//        }

//        std::cout << "Left Arm Pose:" << std::endl;
//        for(int i=0;i<4;i++){
//            for(int j=0;j<4;j++) std::cout << leftArmPose.data[i][j] << " ";
//            std::cout << std::endl;
//        }

//        std::cout << "Right Arm Pose:" << std::endl;
//        for(int i=0;i<4;i++){
//            for(int j=0;j<4;j++) std::cout << rightArmPose.data[i][j] << " ";
//            std::cout << std::endl;
//        }

//        std::cout << "--------------------------" << std::endl;
//    }
    std::string address = "tcp://127.0.0.1:5555";
    auto visionProCollector = VisionProCollector(address);
    visionProCollector.run();

    return 0;
}
