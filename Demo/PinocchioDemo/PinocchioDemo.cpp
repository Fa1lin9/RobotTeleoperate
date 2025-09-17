#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <source_path.h>

#include <IKSolver/IKSolver.hpp>

const std::string modelPath =
        std::string(SOURCE_FILE_PATH)+"/assets/urdf/update_kanuopu-robot.urdf";

int main(){
    IKSolver::BasicConfig config = {
        .type = SolverType::CrpRobot,
        .baseFrameName = {"BASE_S"},
        .targetFrameName = {"L_WRIST_R", "R_WRIST_R"},
        .maxIteration = 400,
        .relativeTol = 1e-4,
    };

    boost::shared_ptr<IKSolver> ikSolverPtr = IKSolver::GetPtr(config);

//    ikSolverPtr->Info();

    Eigen::VectorXd qInit = Eigen::VectorXd::Zero(21);
    qInit.segment(4,7) << -0.72, -1.0, 0.57, -1.0, 0.83, 0, 0;
    qInit.segment(14,7) << 0.72, 1.0, -0.57, 1.0, -0.83, 0, 0;
//    std::cout<<" qInit \n"<<qInit<<std::endl;

    Eigen::Matrix4d leftArmTargetPose,rightArmTargetPose;
    leftArmTargetPose <<     1 , 0 , 0 , 0.3,
                             0 , -1 , 0 , -0.1,
                             0 , 0 , -1 , 0.6,
                             0 , 0 , 0 , 1;
    rightArmTargetPose <<    1 , 0 , 0 , -0.1,
                             0 , 1 , 0 , -0.1,
                             0 , 0 , 1 , 0.6,
                             0 , 0 , 0 , 1;
    std::vector<Eigen::Matrix4d> targetPose = {leftArmTargetPose, rightArmTargetPose};
    std::cout<<" start to solve "<<std::endl;
    ikSolverPtr->Solve(targetPose,qInit,true);

}
