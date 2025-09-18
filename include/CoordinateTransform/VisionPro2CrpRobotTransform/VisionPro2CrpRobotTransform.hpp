#pragma once

#include <CoordinateTransform/CoordinateTransform.hpp>

// 实现坐标系从VisionPro到CrpRobot的转换
class VisionPro2CrpRobotTransform
    : public CoordinateTransform
{
public:
    VisionPro2CrpRobotTransform(const CoordinateTransform::BasicConfig &config_);
    ~VisionPro2CrpRobotTransform();

    std::vector<Eigen::Matrix4d> Transform(const CoordinateTransform::MsgConfig &config_) override;

private:

    Eigen::Matrix<double,4,4>                         T_Head2Waist;//头到机器人腰

    Eigen::Matrix<double,4,4>                         T_XR2Robot;//OpenXR到机器人（旋转角度）
    Eigen::Matrix<double,4,4>                         T_Robot2LeftWrist;//机器人基坐标系到手腕（旋转角度）
    Eigen::Matrix<double,4,4>                         T_Robot2RightWrist;//机器人基坐标系到手腕（旋转角度）

    Eigen::Vector3d                                   offset;
};
