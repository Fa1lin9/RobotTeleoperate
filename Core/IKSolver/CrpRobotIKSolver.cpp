#include "CrpRobotIKSolver.hpp"

CrpRobotIKSolver::CrpRobotIKSolver(const IKSolver::config &config_)
{
    // Init Model
    pinocchio::urdf::buildModel(
                this->modelPath,
                this->robotModel);
}

CrpRobotIKSolver::~CrpRobotIKSolver()
{

}

boost::optional<Eigen::VectorXd> CrpRobotIKSolver::Solve(
        std::vector<Eigen::Matrix4d> targetPose,
        Eigen::VectorXd qInit)
{
    return boost::none;
}

void CrpRobotIKSolver::Info(){
    LOG_FUNCTION;

    std::cout << " ----------------------------------------------------- "<< std::endl;

    // 注意 robotModel.joints包含了全部关节，即包含了初始关节
    // 而整个机器人的自由度是指其可动的关节
    // 所以打印的时候我们可以看到总关节是22，但是DOF确实21
    // 因此，在我们打印相关关节的上下界的时候要注意索引的匹配
    for(pinocchio::JointIndex i=1; i<robotModel.njoints; ++i){
        std::cout << "Joint " << i << ": " << robotModel.names[i]
                  << ", type: " << robotModel.joints[i].shortname()
                  << ", parent: " << robotModel.parents[i] << std::endl;


        std::cout << " lower limit: " << robotModel.lowerPositionLimit[i-1]
                  << ", upper limit: " << robotModel.upperPositionLimit[i-1]<<std::endl;
        std::cout<<std::endl;

    }

    std::cout << " ----------------------------------------------------- "<< std::endl;

//    std::cout << "Size of joints upper limitation: " << robotModel.upperPositionLimit.rows() << std::endl;
    std::cout << "Number of joints: " << robotModel.njoints << std::endl;
    std::cout << "Number of DOFs: " << robotModel.nv << std::endl;
    std::cout << "Number of frames: " << robotModel.nframes << std::endl;

    // Demo
    Eigen::VectorXd q;
//    q.setZero(21);
    q.setOnes(21);
    q[0] = 0;
    q.tail(20) *= 0.1;

    auto start = std::chrono::high_resolution_clock::now();

    pinocchio::Data data=pinocchio::Data(robotModel);
    pinocchio::forwardKinematics(robotModel,data,q);
    pinocchio::updateFramePlacements(robotModel,data);

    // For Left Arm
    pinocchio::JointIndex leftArmStart = robotModel.getJointId("L_SHOULDER_P");
    pinocchio::JointIndex leftArmEnd  = robotModel.getJointId("L_WRIST_R");

    pinocchio::SE3 leftShoulderPitchPose = data.oMi[leftArmStart];
    pinocchio::SE3 leftWaistRollPose  = data.oMi[leftArmEnd];

    pinocchio::SE3 leftArmPose = leftShoulderPitchPose.inverse() * leftWaistRollPose;

    // For Right Arm
    pinocchio::JointIndex rightArmStart = robotModel.getJointId("R_SHOULDER_P");
    pinocchio::JointIndex rightArmEnd= robotModel.getJointId("R_WRIST_R");

    pinocchio::SE3 rightShoulderPitchPose = data.oMi[rightArmStart];
    pinocchio::SE3 rightWaistRollPose  = data.oMi[rightArmEnd];

    pinocchio::SE3 rightArmPose = rightShoulderPitchPose.inverse() * rightWaistRollPose;

    // 打印平移向量
    std::cout << "leftArmPose position: " << leftArmPose.translation().transpose() << std::endl;
    std::cout << "rightArmPose position: "  << rightArmPose.translation().transpose() << std::endl;

    // 打印旋转矩阵
    std::cout << "leftArmPose rotation:\n" << leftArmPose.rotation() << std::endl;
    std::cout << "rightArmPose rotation:\n"  << rightArmPose.rotation() << std::endl;

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "耗时: " << duration.count() << " ms" << std::endl;

}
