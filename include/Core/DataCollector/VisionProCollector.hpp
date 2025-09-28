#pragma once
#include <Eigen/Dense>
#include <zmq.hpp>
#include <iostream>
#include <mutex>

// 打印函数信息
#include <FunctionLogger.hpp>

// 该类主要是负责接受VisionPro端解析得到的数据
// 通信方式主要是通过Socket解析python脚本发送过来的数据
class VisionProCollector
{
public:

    VisionProCollector(std::string address_);
    ~VisionProCollector();
    void run();
    std::vector<Eigen::Matrix4d> GetValue();

private:
    // some variable for socket
    std::string address;
    zmq::context_t context;
    zmq::socket_t subscriber;

    // some variable for data
    // 头、双臂的位姿矩阵
    Eigen::Matrix4d headPose;
    Eigen::Matrix4d leftWristPose;
    Eigen::Matrix4d rightWristPose;

    // 两只手的点位
    Eigen::Matrix<double,25,3> leftHandPosition;
    Eigen::Matrix<double,25,3> rightHandPosition;

    // lock
    std::mutex mutex;

};
