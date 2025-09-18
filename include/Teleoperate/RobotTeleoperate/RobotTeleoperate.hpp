#pragma once

#include <IKSolver/IKSolver.hpp>
#include <PhysicalRobot/PhysicalRobot.hpp>
#include <DataCollector/VisionProCollector.hpp>
#include <CoordinateTransform/CoordinateTransform.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

class RobotTeleoperate
{
public:
    enum Type{
        CrpRobot
    };

    struct BasicConfig{
        RobotTeleoperate::Type type;
    };

    RobotTeleoperate();
    ~RobotTeleoperate();

    static boost::shared_ptr<RobotTeleoperate> GetPtr(const RobotTeleoperate::BasicConfig& config_);

    virtual bool Init() = 0;
    virtual bool StartTeleoperate() = 0;

protected:
    boost::shared_ptr<IKSolver> ikSolverPtr;

    boost::shared_ptr<PhysicalRobot> physicalRobotPtr;

    boost::shared_ptr<CoordinateTransform> transformPtr;

};
