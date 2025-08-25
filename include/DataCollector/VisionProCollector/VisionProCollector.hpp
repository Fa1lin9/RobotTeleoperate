#pragma once
#include <Eigen/Dense>
#include <zmq.hpp>
#include <iostream>

#include <FunctionLogger.hpp>

// 该类主要是负责接受VisionPro端解析得到的数据
// 通信方式主要是通过Socket解析python脚本发送过来的数据
class VisionProCollector
{
public:

    VisionProCollector(std::string address_);
    ~VisionProCollector();
    void run();

private:
    // some variable for socket
    std::string address;
    zmq::context_t context;
    zmq::socket_t subscriber;

    // some variable for data
    // 头、双臂的位姿矩阵
    Eigen::Matrix4d headPose;
    Eigen::Matrix4d leftArmPose;
    Eigen::Matrix4d rightArmPose;

    // 两只手的点位
    Eigen::Matrix<double,25,3> leftHandPosi;
    Eigen::Matrix<double,25,3> rightHandPosi;

};
