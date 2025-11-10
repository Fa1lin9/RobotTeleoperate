#pragma once
#include <RobotTeleoperate/RobotTeleoperate.hpp>

#include <DataCollector/VisionProCollector.hpp>

class Ti5RobotTeleoperate
        :public RobotTeleoperate
{
public:
    Ti5RobotTeleoperate(const RobotTeleoperate::BasicConfig &config);
    ~Ti5RobotTeleoperate();

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
