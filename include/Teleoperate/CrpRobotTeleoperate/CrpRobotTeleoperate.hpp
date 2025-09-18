#pragma once
#include <RobotTeleoperate/RobotTeleoperate.hpp>

#include <DataCollector/VisionProCollector.hpp>

class CrpRobotTeleoperate
        :public RobotTeleoperate
{
public:
    CrpRobotTeleoperate(const RobotTeleoperate::BasicConfig &config);
    ~CrpRobotTeleoperate();

    bool Init() override;
    bool StartTeleoperate() override;

private:
    std::string address;

    VisionProCollector dataCollector;

    Eigen::VectorXd qInit;

};
