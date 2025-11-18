#pragma once
#include <RobotTeleoperate/RobotTeleoperate.hpp>

#include <DataCollector/VisionProCollector.hpp>

<<<<<<< HEAD:include/Teleoperate/CrpRobotTeleoperate/CrpRobotTeleoperate.hpp
#include <thread>
#include <chrono>

class CrpRobotTeleoperate
=======
class Ti5RobotTeleoperate
>>>>>>> 28e193747a6947b3cb96080adcbed78b4c72d09e:include/Teleoperate/Ti5RobotTeleoperate/Ti5RobotTeleoperate.hpp
        :public RobotTeleoperate
{
public:
    Ti5RobotTeleoperate(const RobotTeleoperate::BasicConfig &config);
    Ti5RobotTeleoperate(std::string fileName);
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
