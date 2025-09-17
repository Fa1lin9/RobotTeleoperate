#include "VisionPro2CrpRobotTransform.hpp"
VisionPro2CrpRobotTransform::VisionPro2CrpRobotTransform(const CoordinateTransform::BasicConfig &config_)
    :T_Head2Waist(config_.T_Head2Waist),
     T_XR2Robot(config_.T_XR2Robot),
     T_Robot2LeftWrist(config_.T_Robot2LeftWrist),
     T_Robot2RightWrist(config_.T_Robot2RightWrist),
     offset(config_.offset)
{

}

VisionPro2CrpRobotTransform::~VisionPro2CrpRobotTransform(){

}

std::vector<Eigen::Matrix4d> VisionPro2CrpRobotTransform::Transform(
        const CoordinateTransform::MsgConfig &config_){

    // 相似变化到标准坐标系Robot
    Eigen::Matrix4d head2RobotWorldPose =
            T_XR2Robot * config_.head2xrWorldPose * T_XR2Robot.inverse();
    Eigen::Matrix4d leftWrist2RobotWorldPose =
            T_XR2Robot * config_.leftWrist2xrWorldPose * T_XR2Robot.inverse();
    Eigen::Matrix4d rightWrist2RobotWorldPose =
            T_XR2Robot * config_.rightWrist2xrWorldPose * T_XR2Robot.inverse();

    // 与urdf定义的手腕坐标xyz轴重合
    leftWrist2RobotWorldPose = leftWrist2RobotWorldPose * T_Robot2LeftWrist;
    rightWrist2RobotWorldPose = rightWrist2RobotWorldPose * T_Robot2RightWrist;

    // 转化到人头坐标系
    Eigen::Matrix4d head2RobotWorldPoseInv = head2RobotWorldPose.inverse();
    leftWrist2RobotWorldPose = head2RobotWorldPoseInv * leftWrist2RobotWorldPose;
    rightWrist2RobotWorldPose = head2RobotWorldPoseInv * rightWrist2RobotWorldPose;

    // 补偿
    leftWrist2RobotWorldPose.block<3,1>(0,3) += offset;
    rightWrist2RobotWorldPose.block<3,1>(0,3) += offset;

    // 转化到腰坐标系
    Eigen::Matrix4d LeftWrist2RobotWaistPose = T_Head2Waist * leftWrist2RobotWorldPose;
    Eigen::Matrix4d RightWrist2RobotWaistPose = T_Head2Waist * rightWrist2RobotWorldPose;

    // 实际上最后就是相对于人头的坐标系
    return {LeftWrist2RobotWaistPose , RightWrist2RobotWaistPose};
}
