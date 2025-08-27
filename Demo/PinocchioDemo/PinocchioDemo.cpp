#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <source_path.h>

#include <IKSolver/IKSolver.hpp>

const std::string modelPath =
        std::string(SOURCE_FILE_PATH)+"/assets/urdf/update_kanuopu-robot.urdf";

int main(){
    IKSolver::config config = {
        .type = SolverType::CrpRobot
    };

    boost::shared_ptr<IKSolver> ikSolverPtr = IKSolver::GetPtr(config);

    ikSolverPtr->Info();


}
