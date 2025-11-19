#pragma once
#include <RobotTeleoperate/RobotTeleoperate.hpp>

#include <DataCollector/VisionProCollector.hpp>

class Ti5RobotTeleoperate
        :public RobotTeleoperate
{
public:
    Ti5RobotTeleoperate(const RobotTeleoperate::BasicConfig &config);
    Ti5RobotTeleoperate(const std::string &fileName);
    ~Ti5RobotTeleoperate();

    bool Init() override;
    bool StartTeleoperate() override;
    bool StopTeleoperate() override;
    bool EndTeleoperate() override;

private:
    std::string address;
    CsvWriter csvWriter;

    VisionProCollector dataCollector;
    Ros2Bridge ros2Bridge;

    Eigen::VectorXd qInit;

    size_t FPS;

    // some flags
    bool startFlag = false;

    bool stopFlag = false;

    bool saveFlag = true;

    bool isSim;
    bool isReal;
};
