#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <FunctionLogger.hpp>
#include <iostream>

enum PhysicalRobotType{
    CrpRobot
};

class PhysicalRobot
{
public:
    struct config
    {
        std::string IP;
        PhysicalRobotType type;
    };
    PhysicalRobot();
    ~PhysicalRobot();

    /* ---------------- Connection ---------------- */

    virtual bool Connect() = 0;

    virtual bool Disconnect() = 0;

    virtual bool isConnect() = 0;

    /* ---------------- Connection ---------------- */

    virtual bool EmergencyStop() = 0;

    virtual bool BackToZero() = 0;

    /* ---------------- Get Information ---------------- */

    virtual std::vector<double> GetJointsAngle() = 0;

    // give some basic information of current robot
    virtual void Info() = 0;

    virtual void GetJointsStatus() = 0;

    /* ---------------- Get Information ---------------- */

    virtual bool MoveJ(const std::vector<double> &jointsAngle_) = 0;
    virtual bool MoveL() = 0;

    static boost::shared_ptr<PhysicalRobot> GetPtr(const PhysicalRobot::config &config_);

private:

};
