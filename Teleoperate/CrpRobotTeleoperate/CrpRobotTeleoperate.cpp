#include <CrpRobotTeleoperate/CrpRobotTeleoperate.hpp>

bool CrpRobotTeleoperate::Init(){
    return true;
}

CrpRobotTeleoperate::CrpRobotTeleoperate(const RobotTeleoperate::BasicConfig &config)
    :address(config.address),
     dataCollector(VisionProCollector(this->address))
{
    // IKSolver
    IKSolver::BasicConfig solverConfig = {
        .type = IKSolver::Type::CrpRobot,
        .baseFrameName = {"BASE_S"},
        .targetFrameName = {"L_WRIST_R", "R_WRIST_R"},
        .maxIteration = 400,
        .relativeTol = 1e-3,
    };

    this->ikSolverPtr = IKSolver::GetPtr(solverConfig);

    // qInit
    this->qInit = Eigen::VectorXd::Zero(21);
    qInit.segment(4,7) << -0.72, -1.0, 0.57, -1.0, 0.83, 0, 0;
    qInit.segment(14,7) << 0.72, 1.0, -0.57, 1.0, -0.83, 0, 0;

    // CoordinateTransform
    Eigen::Matrix4d temp;
    temp<<1,0,0,0,
         0,-1,0,0,
         0,0,-1,0,
         0,0,0,1;
    CoordinateTransform::BasicConfig transformConfig;
    transformConfig.type = CoordinateTransform::Type::VisionPro2CrpRobot;
    transformConfig.T_Head2Waist = Eigen::Matrix4d::Identity();
    transformConfig.T_XR2Robot <<  -1, 0, 0, 0,
                                0, 1, 0, 0,
                                0, 0, -1, 0,
                                0, 0, 0, 1;
    transformConfig.T_Robot2LeftWrist <<0.0, 1.0, 0.0, 0.0,
                                    0.0, 0.0,-1.0, 0.0,
                                   -1.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 1.0;
    transformConfig.T_Robot2LeftWrist = Eigen::Matrix4d::Identity();
    transformConfig.T_Robot2LeftWrist = temp * transformConfig.T_Robot2LeftWrist;
    transformConfig.T_Robot2RightWrist <<   0.0,-1.0, 0.0, 0.0,
                                        0.0, 0.0, 1.0, 0.0,
                                       -1.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 1.0;
    transformConfig.T_Robot2RightWrist = Eigen::Matrix4d::Identity();
    transformConfig.offset << 0, 0, 0.2;
    this->transformPtr = CoordinateTransform::GetPtr(transformConfig);

    // PhysicalRobot
    PhysicalRobot::BasicConfig robotConfig = {
        .type = PhysicalRobot::Type::CrpRobot,
    };

    boost::shared_ptr<PhysicalRobot> physicalRobotPtr
            = PhysicalRobot::GetPtr(robotConfig);
}

CrpRobotTeleoperate::~CrpRobotTeleoperate(){

}

bool CrpRobotTeleoperate::StartTeleoperate(){
    while(1){
        std::vector<Eigen::Matrix4d> msg = this->dataCollector.GetValue();

        if(msg.size()==0 || msg.empty()){
            std::string error = " The message received is not qualified !";
            throw std::invalid_argument(error);
            continue;
        }

        if(0){
            std::cout << "--------------------------" << std::endl;
            std::cout << "Head Pose:\n" << msg[0] << std::endl;
            std::cout << "Left Wrist Pose:\n" << msg[1] << std::endl;
            std::cout << "Right Wrist Pose:\n" << msg[2] << std::endl;
            std::cout << "--------------------------" << std::endl;
        }

        CoordinateTransform::MsgConfig msgConfig{
            .head2xrWorldPose = msg[0],
            .leftWrist2xrWorldPose = msg[1],
            .rightWrist2xrWorldPose = msg[2],
        };

        std::vector<Eigen::Matrix4d> transformedMsg = this->transformPtr->Transform(msgConfig);

        if(1){
            std::cout << "--------------------------" << std::endl;
            std::cout << "Transformed Left Wrist Pose:\n" << transformedMsg[0] << std::endl;
            std::cout << "Transformed Right Wrist Pose:\n" << transformedMsg[1] << std::endl;
            std::cout << "--------------------------" << std::endl;
        }

        std::cout<<"-------------- Start to solve --------------"<<std::endl;

        boost::optional<Eigen::VectorXd> q = ikSolverPtr->Solve(transformedMsg, qInit, false);
        if(q.has_value()){
            qInit = q.value();
            std::cout << "q:\n" << q << std::endl;
        }else{
            continue;
        }

        std::cout<<"---------------- Solver over ----------------"<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return true;
}
