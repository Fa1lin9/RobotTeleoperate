#pragma once
#include <IKSolver/IKSolver.hpp>

class OptimIKSolver
        :public IKSolver
{
public:
    OptimIKSolver(const IKSolver::config &config_);
    ~OptimIKSolver();

    boost::optional<Eigen::VectorXd> Solve(Eigen::VectorXd qInit) override;
private:

};
