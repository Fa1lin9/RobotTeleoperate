#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

enum PhysicalRobotType{
    CrpRobot
};

class PhysicalRobot
{
public:
    struct config
    {
        config() {}

        PhysicalRobotType type;
    };
    PhysicalRobot();
    ~PhysicalRobot();

    static boost::shared_ptr<PhysicalRobot> GetPtr(const PhysicalRobot::config &config_);

private:

};
