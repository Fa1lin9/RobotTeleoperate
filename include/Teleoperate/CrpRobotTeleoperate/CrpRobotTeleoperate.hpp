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
    bool StopTeleoperate() override;
    bool EndTeleoperate() override;

private:
    std::string address;
    CsvWriter csvWriter;

    VisionProCollector dataCollector;

    Eigen::VectorXd qInit;

    // some flags
    bool startFlag = false;

    bool stopFlag = false;

    bool saveFlag = true;
};
