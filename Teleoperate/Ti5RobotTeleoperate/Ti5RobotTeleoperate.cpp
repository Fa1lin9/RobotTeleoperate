#include <Ti5RobotTeleoperate/Ti5RobotTeleoperate.hpp>

bool Ti5RobotTeleoperate::Init(){
    return true;
}

Ti5RobotTeleoperate::Ti5RobotTeleoperate(const RobotTeleoperate::BasicConfig &config)
    :address(config.address),
     dataCollector(VisionProCollector(this->address))
{
    // IKSolver
    Eigen::Matrix4d baseOffset;
    baseOffset << 1, 0, 0, +0.02,
                    0, 1, 0, 0,
                    0, 0, 1, +1.10,
                    0, 0 ,0, 1;
    IKSolver::BasicConfig solverConfig = {
        .type = IKSolver::Type::CrpRobot,
        .baseFrameName = {"BASE_S"},
        .targetFrameName = {"L_WRIST_R", "R_WRIST_R"},
        .baseOffset = {baseOffset},
        .maxIteration = 400,
        .relativeTol = 1e-2,
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
    transformConfig.type = CoordinateTransform::Type::VisionPro2Ti5Robot;
    transformConfig.T_Head2Waist = Eigen::Matrix4d::Identity();
    transformConfig.T_XR2Robot <<   0, 0, -1, 0,
                                    -1, 0, 0, 0,
                                    0, 1, 0, 0,
                                    0, 0, 0, 1;
    transformConfig.T_Robot2LeftWrist <<0.0, 1.0, 0.0, 0.0,
                                        -1.0, 0.0,0.0, 0.0,
                                        0.0, 0.0, 1.0, 0.0,
                                        0.0, 0.0, 0.0, 1.0;
//    transformConfig.T_Robot2LeftWrist = Eigen::Matrix4d::Identity();
//    transformConfig.T_Robot2LeftWrist = temp * transformConfig.T_Robot2LeftWrist;
    transformConfig.T_Robot2RightWrist <<   0.0,-1.0, 0.0, 0.0,
                                            1.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 1.0, 0.0,
                                            0.0, 0.0, 0.0, 1.0;
//    transformConfig.T_Robot2RightWrist = Eigen::Matrix4d::Identity();
    transformConfig.offset << 0, 0, 0;
    this->transformPtr = CoordinateTransform::GetPtr(transformConfig);

    // PhysicalRobot
    PhysicalRobot::BasicConfig robotConfig = {
        .type = PhysicalRobot::Type::CrpRobot,
    };

    boost::shared_ptr<PhysicalRobot> physicalRobotPtr
            = PhysicalRobot::GetPtr(robotConfig);
}

Ti5RobotTeleoperate::~Ti5RobotTeleoperate(){

}

bool Ti5RobotTeleoperate::StartTeleoperate(){

    int FPS = 20;
    this->startFlag = true;

    while(this->startFlag){
        if(this->stopFlag){
            std::cout<<"Teleoperation Stop ! "<<std::endl;
            continue;
        }

        auto start = std::chrono::high_resolution_clock::now();
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

//        auto solveStart = std::chrono::high_resolution_clock::now();
        boost::optional<Eigen::VectorXd> q = ikSolverPtr->Solve(transformedMsg, qInit, true);
//        auto solveEnd = std::chrono::high_resolution_clock::now();
//        auto solveDuration = std::chrono::duration_cast<std::chrono::milliseconds>(solveEnd - solveStart);
//        std::cout << " Solve 耗时: " << solveDuration.count() << " ms" << std::endl;
        if(q.has_value()){
            qInit = q.value();
//            qInit = physicalRobotPtr->GetJointsAngleEigen();
            std::cout << "q:\n" << q << std::endl;
        }else{
            std::cout<<" Solve failed! "<<std::endl;
            continue;
        }

        // save data to log
        if(this->saveFlag){
            this->csvWriter.WriteEigenVector(q.value());
        }
        std::cout<<"---------------- Solver over ----------------"<<std::endl;

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << " Main Loop 耗时: " << duration.count() << " ms" << std::endl;


        int framePeriod = static_cast<int>(1000.0 / FPS);
        int sleepTime = framePeriod - duration.count();

        if(sleepTime > 0){
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
        }
    }

    return true;
}

bool Ti5RobotTeleoperate::StopTeleoperate(){
    this->stopFlag = true;
    return true;
}

bool Ti5RobotTeleoperate::EndTeleoperate(){
    if(!this->startFlag){
        std::cout<<"Teleoperation has ended! "<<std::endl;
    }else{
        this->startFlag = false;
    }

    return true;
}
