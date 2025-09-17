#pragma once
#include <IKSolver/IKSolver.hpp>
#include <source_path.h>

#include <chrono>

/* ------------------ Details of the Joints ------------------

*Joint 1: UP-DOWN, type: JointModelPZ, parent: 0
 lower limit: 0, upper limit: 0.225

*Joint 2: WAIST_R, type: JointModelRX, parent: 1
 lower limit: -0.34907, upper limit: 0.43633

*Joint 3: WAIST_Y, type: JointModelRZ, parent: 2
 lower limit: -0.17453, upper limit: 0.17453

*Joint 4: WAIST_P, type: JointModelRevoluteUnaligned, parent: 3
 lower limit: -0.5236, upper limit: 0.5236

*Joint 5: L_SHOULDER_P, type: JointModelRY, parent: 4
 lower limit: -3.1416, upper limit: 3.1416

*Joint 6: L_SHOUDLER_R, type: JointModelRX, parent: 5
 lower limit: -1.5708, upper limit: 0.5236

*Joint 7: L_SHOULDER_Y, type: JointModelRY, parent: 6
 lower limit: -3.1416, upper limit: 3.1416

*Joint 8: L_ELBOW_Y, type: JointModelRX, parent: 7
 lower limit: -2.0071, upper limit: 0

*Joint 9: L_WRIST_P, type: JointModelRY, parent: 8
 lower limit: -3.1416, upper limit: 3.1416

*Joint 10: L_WRIST_Y, type: JointModelRZ, parent: 9
 lower limit: -1.3963, upper limit: 1.3963

*Joint 11: L_WRIST_R, type: JointModelRX, parent: 10
 lower limit: -1.5533, upper limit: 0.08727

*Joint 12: NECK_Y, type: JointModelRZ, parent: 4
 lower limit: -1.5708, upper limit: 1.5708

*Joint 13: NECK_P, type: JointModelRevoluteUnaligned, parent: 12
 lower limit: -0.5236, upper limit: 0.69813

*Joint 14: NECK_R, type: JointModelRX, parent: 13
 lower limit: -0.2618, upper limit: 0.2618

*Joint 15: R_SHOULDER_P, type: JointModelRevoluteUnaligned, parent: 4
 lower limit: -3.1416, upper limit: 3.1416

*Joint 16: R_SHOULDER_R, type: JointModelRX, parent: 15
 lower limit: -0.5236, upper limit: 1.5708

*Joint 17: R_SHOULDER_Y, type: JointModelRevoluteUnaligned, parent: 16
 lower limit: -3.1416, upper limit: 3.1416

*Joint 18: R_ELBOW_Y, type: JointModelRX, parent: 17
 lower limit: 0, upper limit: 2.0071

*Joint 19: R_WRIST_P, type: JointModelRevoluteUnaligned, parent: 18
 lower limit: -3.1416, upper limit: 3.1416

*Joint 20: R_WRIST_Y, type: JointModelRZ, parent: 19
 lower limit: -1.3963, upper limit: 1.3963

*Joint 21: R_WRIST_R, type: JointModelRX, parent: 20
 lower limit: 0, upper limit: 1.5533

 ------------------ Details of the Joints ------------------ */

class CrpRobotIKSolver
        :public IKSolver
{
public:
    CrpRobotIKSolver(const IKSolver::BasicConfig &config_);
    ~CrpRobotIKSolver();

    boost::optional<Eigen::VectorXd> Solve(
                    const std::vector<Eigen::Matrix4d>& targetPose,
                    const Eigen::VectorXd& qInit) override;

    std::vector<pinocchio::SE3> Forward(const Eigen::VectorXd& q) override;

    Eigen::VectorXd GetGradient(const IKSolver::CrpRobotConfig& config_);

    void Info() override;

    /* ------------------ Basic Info ------------------ */
    // the degree of freedom of CRP's Robot
    // 一只手7个自由度
    // 头部3个自由度
    // 腰部3个自由度
    // AGV处有个UP_DOWN关节，是沿着Z轴的平移关节
    // 总共 7 * 2 + 3 + 3 + 1 = 21
    const size_t dofTotal = 21;

    const size_t dofArm = 7;

private:
    struct CrpRobotData{
        CrpRobotIKSolver *solver;
        Eigen::VectorXd qInit;
        std::vector<Eigen::Matrix4d> targetPose;
//        Eigen::Matrix4d leftArmTargetPose;
//        Eigen::Matrix4d rightArmTargetPose;
    };

private:
    double CostFunc(const IKSolver::CrpRobotConfig& config_);

    casadi::SX CostFuncAD(const pinocchio::ModelTpl<casadi::SX>::ConfigVectorType& q,
                        const Eigen::Matrix<casadi::SX,Eigen::Dynamic,1>& qInit,
                        const std::vector<Eigen::Matrix<casadi::SX,4,4>>& targetPose
                        );

    Eigen::VectorXd GradFunc(const IKSolver::CrpRobotConfig& config_);

    void NormalizeAngle(Eigen::VectorXd& angle);

    void Initialize();

    void InitializeAD(const std::vector<Eigen::Matrix4d>& targetPose,
                      const Eigen::VectorXd& qInit);

    // the urdf file path of the CRP's Robot
    const std::string modelPath =
            std::string(SOURCE_FILE_PATH) + "/assets/urdf/update_kanuopu-robot.urdf";

    /* ------------------ Robot Parameter ------------------ */

    pinocchio::Model robotModel;

    std::vector<size_t> leftArmID = {5,6,7,8,9,10,11};

    std::vector<size_t> rightArmID = {15,16,17,18,19,20,21};

    size_t baseIndex;
    size_t leftArmEndIndex;
    size_t rightArmEndIndex;

    std::vector<double> qLeftArmNeutral;
    std::vector<double> qRightArmNeutral;
    std::vector<double> qNeutral;

    std::vector<double> leftArmBoundsLower;
    std::vector<double> rightArmBoundsLower;

    std::vector<double> leftArmBoundsUpper;
    std::vector<double> rightArmBoundsUpper;

    std::vector<double> totalBoundsLower;
    std::vector<double> totalBoundsUpper;

    /* ------------------ Robot Parameter ------------------ */

    /* ------------------ Casadi Auto-Diff ------------------ */

    pinocchio::ModelTpl<casadi::SX> robotModelAD;

    // Function
    casadi::Function mainFunc;



    /* ------------------ Casadi Auto-Diff ------------------ */

};
