#include <DataCollector/VisionProCollector.hpp>

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
    leftWristPose.setZero();
    leftWristPose(3,3) = 1;
    rightWristPose.setZero();
    rightWristPose(3,3) = 1;

    leftHandPosition.setZero();
    rightHandPosition.setZero();

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

        {
            std::lock_guard<std::mutex> lock(mutex);

            // Get Data
            std::memcpy(this->headPose.data(), msg.data(), sizeof(Eigen::Matrix4d));
            std::memcpy(this->leftWristPose.data(), (char*)msg.data() + sizeof(Eigen::Matrix4d), sizeof(Eigen::Matrix4d));
            std::memcpy(this->rightWristPose.data(), (char*)msg.data() + sizeof(Eigen::Matrix4d) * 2, sizeof(Eigen::Matrix4d));

            // Transpose
            this->headPose.transposeInPlace();
            this->leftWristPose.transposeInPlace();
            this->rightWristPose.transposeInPlace();

        }

        // Matrix Output
        if(0){
            std::cout << "--------------------------" << std::endl;
            std::cout << "Head Pose:\n" << headPose << std::endl;
            std::cout << "Left Wrist Pose:\n" << leftWristPose << std::endl;
            std::cout << "Right Wrist Pose:\n" << rightWristPose << std::endl;
            std::cout << "--------------------------" << std::endl;
        }

    }

}

std::vector<Eigen::Matrix4d> VisionProCollector::GetValue(){
//    LOG_FUNCTION;
    zmq::message_t msg;
    subscriber.recv(msg, zmq::recv_flags::none);

    if (msg.size() != sizeof(Eigen::Matrix4d) * 3) {
        std::cout<<"The length should be :"<<sizeof(Eigen::Matrix4d) * 3<<std::endl;
        std::cerr << "Received wrong size: " << msg.size() << std::endl;
        return {};
    }

    // Get Data
    std::memcpy(this->headPose.data(), msg.data(), sizeof(Eigen::Matrix4d));
    std::memcpy(this->leftWristPose.data(), (char*)msg.data() + sizeof(Eigen::Matrix4d), sizeof(Eigen::Matrix4d));
    std::memcpy(this->rightWristPose.data(), (char*)msg.data() + sizeof(Eigen::Matrix4d) * 2, sizeof(Eigen::Matrix4d));

    // Transpose
    this->headPose.transposeInPlace();
    this->leftWristPose.transposeInPlace();
    this->rightWristPose.transposeInPlace();

    return {this->headPose, this->leftWristPose, this->rightWristPose};
}
