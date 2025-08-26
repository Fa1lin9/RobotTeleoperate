#include <IKSolver/OptimIKSolver.hpp>

OptimIKSolver::OptimIKSolver(const IKSolver::config &config_)
{

}

OptimIKSolver::~OptimIKSolver()
{

}

boost::optional<Eigen::VectorXd> OptimIKSolver::Solve(Eigen::VectorXd qInit)
{
    return boost::none;
}
