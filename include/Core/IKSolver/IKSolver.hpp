#pragma once

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <boost/make_shared.hpp>
#include <boost/optional.hpp>
#include <iostream>
#include <FunctionLogger.hpp>

#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/autodiff/casadi.hpp>

#include <casadi/casadi.hpp>

#include <nlopt.hpp>

enum SolverType{
    CrpRobot
};

class IKSolver
{
public:
    struct config
    {
        std::string modelPath;

        SolverType type;

        std::vector<std::string> baseFrameName;
        std::vector<std::string> targetFrameName;
    };

    struct CrpRobotConfig{

        Eigen::VectorXd q;
        Eigen::VectorXd qInit;
        // 2 pose: left arm and right arm
        std::vector<Eigen::Matrix4d> targetPose;
//        Eigen::Matrix4d leftArmTargetPose;
//        Eigen::Matrix4d rightArmTargetPose;

    };

    IKSolver();
    ~IKSolver();

    // Solver the IK
    //考虑到目标位姿包含双臂末端，同时考虑到泛化性，所以用std::vector
    virtual boost::optional<Eigen::VectorXd> Solve(
                    const std::vector<Eigen::Matrix4d>& targetPose,
                    const Eigen::VectorXd& qInit) = 0;

    virtual std::vector<pinocchio::SE3> Forward(const Eigen::VectorXd& q) = 0;

    // Output some information of the current solver
    virtual void Info() = 0;

    static boost::shared_ptr<IKSolver> GetPtr(const IKSolver::config& config_);


private:

};
