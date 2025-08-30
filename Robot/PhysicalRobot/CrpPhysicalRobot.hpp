#pragma once
#include <PhysicalRobot/PhysicalRobot.hpp>

#include <stdio.h>
#include <cstring>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <cmath>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <Ti5MOVE.h>
#include <Ti5BASIC.h>
#include <Ti5LOGIC.h>
#include <mathfunc.h>
#include <tool.h>
#include <csignal>
#include <mutex>
#include <shared_mutex>
#include <Ti5CAN_Driver.h>
#include <map>
#include <termios.h>
#include <fcntl.h>
#include <algorithm>
#include <sys/socket.h>
#include <arpa/inet.h>

#define RESET "\033[0m"
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */

enum ArmType{
    LeftArm,
    RightArm,
};

class CrpPhysicalRobot
        :public PhysicalRobot
{
public:
    CrpPhysicalRobot(const PhysicalRobot::config &config_);

    ~CrpPhysicalRobot();


    /* ---------------- Connection ---------------- */

    bool Connect() override;

    bool Disconnect() override;

    bool isConnect() override;

    /* ---------------- Connection ---------------- */

    bool EmergencyStop() override;

    bool BackToZero() override;

    std::vector<double> GetJointsAngle() override;
private:

    bool connectFlag = false;

};
