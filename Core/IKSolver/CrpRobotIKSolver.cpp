#include "CrpRobotIKSolver.hpp"

CrpRobotIKSolver::CrpRobotIKSolver(const IKSolver::config &config_)
{
    // Init Model
    pinocchio::urdf::buildModel(
                this->modelPath,
                this->robotModel);

    // As For CrpRobot
    // The size of the baseFrameName should be 1
    // The size of the targetFrameName should be 2, and as the order: left arm , right arm
    if(config_.baseFrameName.size() != 1){
        std::string error = " As for CrpRobotIKSolver,the size of the baseFrameName should be 1! ";
        throw std::length_error(error);
    }else{
        this->baseIndex = this->robotModel.getFrameId(config_.baseFrameName[0]);
    }

    if(config_.targetFrameName.size() != 2){
        std::string error = " As for CrpRobotIKSolver,the size of the baseFrameName should be 2! And the order is: left arm, right arm! ";
        throw std::length_error(error);
    }else{
        this->leftArmEndIndex = this->robotModel.getFrameId(config_.targetFrameName[0]);
        this->rightArmEndIndex = this->robotModel.getFrameId(config_.targetFrameName[1]);
    }
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

std::vector<pinocchio::SE3> CrpRobotIKSolver::Forward(const Eigen::VectorXd& q){
    if(q.size() != this->robotModel.nq){
        std::string error = " The size of the q should be this->robotModel.nq! ";
        throw std::length_error(error);
    }

    // updata data to better get position
    pinocchio::Data data=pinocchio::Data(robotModel);
    pinocchio::forwardKinematics(robotModel,data,q);
    pinocchio::updateFramePlacements(robotModel,data);

    // base pose
    pinocchio::SE3 basePose = data.oMi[this->baseIndex];

    // left arm
    pinocchio::SE3 leftArmEndPose = data.oMi[this->leftArmEndIndex];
    pinocchio::SE3 leftArmPose = basePose.inverse() * leftArmEndPose;


    // right arm
    pinocchio::SE3 rightArmEndPose = data.oMi[this->rightArmEndIndex];
    pinocchio::SE3 rightArmPose = basePose.inverse() * rightArmEndPose;

    return std::vector<pinocchio::SE3>{leftArmPose, rightArmPose};
}

double CrpRobotIKSolver::CostFunc(const IKSolver::CostFuncConfig& config_){
    std::vector<pinocchio::SE3> currentPose = this->Forward(config_.q);

    // translation error
    Eigen::Vector3d currentLeftTrans  = currentPose[0].translation();
    Eigen::Vector3d currentRightTrans = currentPose[1].translation();
    Eigen::Vector3d targetLeftTrans   = config_.targetPose[0].block<3,1>(0,3);
    Eigen::Vector3d targetRightTrans  = config_.targetPose[1].block<3,1>(0,3);

    Eigen::VectorXd currentTrans(6), targetTrans(6);
    currentTrans << currentLeftTrans, currentRightTrans;
    targetTrans  << targetLeftTrans, targetRightTrans;

    Eigen::VectorXd transErrorVec = currentTrans - targetTrans;
    double transError = transErrorVec.norm();

    // rotation error
    Eigen::Matrix3d leftArmError
            = config_.targetPose[0].block<3,3>(0,0) * currentPose[0].rotation().inverse();
    Eigen::Matrix3d rightArmError
            = config_.targetPose[1].block<3,3>(0,0) * currentPose[1].rotation().inverse();

    Eigen::VectorXd rotaErrorVec(6);
    rotaErrorVec << pinocchio::log3(leftArmError), pinocchio::log3(rightArmError);
    double rotaError = rotaErrorVec.norm();

    // smoothing error
    Eigen::VectorXd smoothErrorVec = config_.q - config_.qInit;
    this->NormalizeAngle(smoothErrorVec);
    double smoothError = smoothErrorVec.norm();




}

void CrpRobotIKSolver::NormalizeAngle(Eigen::VectorXd& angle){
    for(int i=0;i<angle.size();i++){
        angle(i) = fmod(angle(i) + M_PI, 2 * M_PI); // 先 +π 再取模
        if (angle(i) < 0) {
            angle(i) += 2 * M_PI; // 确保在 [0, 2π]
        }
        angle(i) -= M_PI; // 回到 [-π, π]
    }
}
