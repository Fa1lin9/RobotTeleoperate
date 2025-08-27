#pragma once

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <boost/make_shared.hpp>
#include <boost/optional.hpp>
#include <iostream>
#include <FunctionLogger.hpp>

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
    };
    IKSolver();
    ~IKSolver();

    // Solver the IK
    //考虑到目标位姿包含双臂末端，同时考虑到泛化性，所以用std::vector
    virtual boost::optional<Eigen::VectorXd> Solve(
                    std::vector<Eigen::Matrix4d> targetPose,
                    Eigen::VectorXd qInit) = 0;

    // Output some information of the current solver
    virtual void Info() = 0;

    static boost::shared_ptr<IKSolver> GetPtr(const IKSolver::config &config_);


private:

};
