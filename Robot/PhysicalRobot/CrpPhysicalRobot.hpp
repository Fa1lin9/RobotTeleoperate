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
    /* ---------------- Basic Information ---------------- */

    // Left(device0): 23, 24, 25, 26, 27, 28, 29
    // Right(device1): 16, 17, 18, 19, 14, 21, 22
    // Head(device0): 30, 31, 32
    // name: {can_device, can_id, index}

    // {"left_joint_3", {0, 25, 18}},
    // {"left_joint_4", {0, 26, 19}},
    // {"left_joint_5", {0, 27, 20}},
    // {"left_joint_6", {0, 28, 21}},
    // {"left_joint_7", {0, 29, 22}},
    // {"right_joint_1", {0, 16, 23}},
    // {"right_joint_2", {0, 17, 24}},
    // {"right_joint_3", {0, 18, 25}},
    // {"right_joint_4", {0, 19, 26}},
    // {"right_joint_5", {0, 20, 27}},
    // {"right_joint_6", {0, 21, 28}},
    // {"right_joint_7", {0, 22, 29}},
    // {"head_joint_1", {0, 30, 30}},
    // {"head_joint_2", {0, 31, 31}},
    // {"head_joint_3", {0, 32, 32}},

    /* ---------------- Basic Information ---------------- */

    /* ---------------- Connection ---------------- */

    bool Connect() override;

    bool Disconnect() override;

    bool isConnect() override;

    /* ---------------- Connection ---------------- */

    bool EmergencyStop() override;

    bool BackToZero() override;

    /* ---------------- Get Information ---------------- */

    void Info() override;

    void GetJointsStatus() override;

    /* ---------------- Get Information ---------------- */

    bool MoveJ(const std::vector<double> &jointsAngle_) override;

    bool MoveL() override;

    std::vector<double> GetJointsAngle() override;
private:

    bool connectFlag = false;

    // degree of freedom
    const size_t dofHead = 3;
    const size_t dofArm = 7;

    //
    std::vector<size_t> headCanIndex = {30, 31, 32};
    std::vector<size_t> leftArmCanIndex = {23, 24, 25, 26, 27, 28, 29};
    std::vector<size_t> rightArmCanIndex = {16, 17, 18, 19, 20, 21, 22};

};
