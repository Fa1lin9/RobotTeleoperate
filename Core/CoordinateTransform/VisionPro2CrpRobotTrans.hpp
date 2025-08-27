#pragma once

#include <CoordinateTransform/CoordinateTransform.hpp>

// 实现坐标系从VisionPro到CrpRobot的转换
class VisionPro2CrpRobotTrans
    : public CoordinateTransform
{
public:
    VisionPro2CrpRobotTrans(const CoordinateTransform::config &config_);
    ~VisionPro2CrpRobotTrans();

    std::vector<Eigen::Matrix4d> GetResult(const CoordinateTransform::config &config_) override;

private:
};
