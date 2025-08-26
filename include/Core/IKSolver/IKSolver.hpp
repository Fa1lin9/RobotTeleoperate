#pragma once

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <boost/make_shared.hpp>
#include <boost/optional.hpp>
#include <iostream>

enum SolverType{
    Optimization
};

class IKSolver
{
public:
    struct config
    {
        config() {}

        std::string modelPath;

        SolverType type;
    };
    IKSolver();
    ~IKSolver();


    virtual boost::optional<Eigen::VectorXd> Solve(Eigen::VectorXd qInit) = 0;

    static boost::shared_ptr<IKSolver> GetPtr(const IKSolver::config &config_);


private:

};
