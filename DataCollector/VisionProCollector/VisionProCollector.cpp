#include <VisionProCollector/VisionProCollector.hpp>

VisionProCollector::VisionProCollector(std::string address_)
     :address(address_),
      context(1),
      subscriber(context, zmq::socket_type::sub)
{
//    LOG_FUNCTION;
    subscriber.connect(address);
    subscriber.set(zmq::sockopt::subscribe, "");  // 订阅所有消息

    // Mat Init
    headPose.setZero();
    headPose(3,3) = 1;
    leftArmPose.setZero();
    leftArmPose(3,3) = 1;
    rightArmPose.setZero();
    rightArmPose(3,3) = 1;

    leftHandPosi.setZero();
    rightHandPosi.setZero();

    std::cout << FUNC_SIG <<" initialized, connected to " << address << std::endl;
}

VisionProCollector::~VisionProCollector(){

}

void VisionProCollector::run(){
    LOG_FUNCTION;

    while (true) {
        zmq::message_t msg;
        subscriber.recv(msg, zmq::recv_flags::none);

        if (msg.size() != sizeof(Eigen::Matrix4d) * 3) {
            std::cout<<"The length should be :"<<sizeof(Eigen::Matrix4d) * 3<<std::endl;
            std::cerr << "Received wrong size: " << msg.size() << std::endl;
            continue;
        }

        // memcpy 到 Eigen 矩阵
        std::memcpy(headPose.data(), msg.data(), sizeof(Eigen::Matrix4d));
        std::memcpy(leftArmPose.data(), (char*)msg.data() + sizeof(Eigen::Matrix4d), sizeof(Eigen::Matrix4d));
        std::memcpy(rightArmPose.data(), (char*)msg.data() + sizeof(Eigen::Matrix4d) * 2, sizeof(Eigen::Matrix4d));

        // 打印矩阵
        std::cout << "Head Pose:\n" << headPose << std::endl;
        std::cout << "Left Arm Pose:\n" << leftArmPose << std::endl;
        std::cout << "Right Arm Pose:\n" << rightArmPose << std::endl;
        std::cout << "--------------------------" << std::endl;
    }

}
