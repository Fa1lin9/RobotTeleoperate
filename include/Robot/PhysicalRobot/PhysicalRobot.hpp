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

    struct CrpRobotConfig
    {
        // some flag
        bool useLeftArm = false;
        bool useRightArm = false;
        bool useHead = false;
        bool useWaist = false;

        //
        std::vector<double> leftArmJointsValue;
        std::vector<double> rightArmJointsValue;
        std::vector<double> headJointsValue;
        std::vector<double> waistJointsValue;
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

    virtual bool BackToZero(const PhysicalRobot::CrpRobotConfig& config_) = 0;

    /* ---------------- Get Information ---------------- */

    virtual std::vector<double> GetJointsAngle() = 0;

    // give some basic information of current robot
    virtual void Info() = 0;

    virtual void GetJointsStatus() = 0;

    /* ---------------- Get Information ---------------- */

    virtual bool MoveJ(const std::vector<double> &jointsAngle_) = 0; 
    virtual bool MoveL() = 0;

    virtual bool MoveJ(const PhysicalRobot::CrpRobotConfig& config_) = 0;

    static boost::shared_ptr<PhysicalRobot> GetPtr(const PhysicalRobot::config &config_);

private:

};
