#pragma once

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <boost/make_shared.hpp>

enum TransType{
    VisionPro2CrpRobot
};

class CoordinateTransform
{
public:
    struct config
    {
        config() {}
        // 头和双臂手腕基于XR设备世界坐标系的位姿矩阵
        Eigen::Matrix4d                 head2xrWorldPose;
        Eigen::Matrix4d                 LeftWrist2xrWorldPose;
        Eigen::Matrix4d                 rightWrist2xrWorldPose;

        // 双手的局部坐标系，包含25个点
        Eigen::Matrix<double,25,3>      leftHandPose;
        Eigen::Matrix<double,25,3>      rightHandPose;

        TransType type;
    };

    CoordinateTransform();

    ~CoordinateTransform();

    virtual std::vector<Eigen::Matrix4d> GetResult(const CoordinateTransform::config &config_) = 0;

    static boost::shared_ptr<CoordinateTransform> GetPtr(const CoordinateTransform::config &config_);

private:

};
