#include <Ti5RobotTeleoperate/Ti5RobotTeleoperate.hpp>

bool Ti5RobotTeleoperate::Init(){
    return true;
}

Ti5RobotTeleoperate::Ti5RobotTeleoperate(const RobotTeleoperate::BasicConfig &config)
    :address(config.address),
     dataCollector(VisionProCollector(this->address)),
     isSim(config.isSim),
     isReal(config.isReal),
     FPS(config.FPS)
{
    // qInit
    this->qInit = Eigen::VectorXd::Zero(21);
    qInit.segment(4,7) << -0.72, -1.0, 0.57, -1.0, 0.83, 0, 0;
    qInit.segment(14,7) << 0.72, 1.0, -0.57, 1.0, -0.83, 0, 0;

    this->ikSolverPtr = IKSolver::GetPtr(config.solverConfig);

    this->transformPtr = CoordinateTransform::GetPtr(config.transformConfig);

    this->physicalRobotPtr = PhysicalRobot::GetPtr(config.robotConfig);

    this->ros2Bridge.Init(config.bridgeConfig);
}

Ti5RobotTeleoperate::~Ti5RobotTeleoperate(){

}

bool Ti5RobotTeleoperate::StartTeleoperate(){
    // Filter
    WeightedMovingFilter filter(std::vector<double>{0.4, 0.3, 0.2, 0.1}, this->ikSolverPtr->GetDofTotal());

//    int FPS = 25;
    this->startFlag = true;
    this->saveFlag = false;

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

        if(0){
            std::cout << "--------------------------" << std::endl;
            std::cout << "Transformed Left Wrist Pose:\n" << transformedMsg[0] << std::endl;
            std::cout << "Transformed Right Wrist Pose:\n" << transformedMsg[1] << std::endl;
            std::cout << "--------------------------" << std::endl;
        }

        std::cout<<"-------------- Start to solve --------------"<<std::endl;

//        auto solveStart = std::chrono::high_resolution_clock::now();
        boost::optional<Eigen::VectorXd> q = ikSolverPtr->Solve(transformedMsg, qInit, false);
//        auto solveEnd = std::chrono::high_resolution_clock::now();
//        auto solveDuration = std::chrono::duration_cast<std::chrono::milliseconds>(solveEnd - solveStart);
//        std::cout << " Solve 耗时: " << solveDuration.count() << " ms" << std::endl;

        Eigen::VectorXd qEigen;

        if(q.has_value()){
            qEigen = q.value();

            // Filter
            filter.AddData(qEigen);
            qEigen = filter.GetFilteredData();

            if(this->isSim){
                // send to ros2
                ti5_interfaces::msg::JointStateWithoutStamp msg;
                std::vector<double> qVec(qEigen.data(), qEigen.data() + qEigen.size());
    //            msg.position() = qVec;
                // Temp send 14 joints value just to apply to GunmpGan's current version
                std::vector<double> qVecOnlyArm;
                qVecOnlyArm.reserve(14);
                qVecOnlyArm.insert(qVecOnlyArm.end(),qVec.begin() + 4, qVec.begin() + 4 + 7);
                qVecOnlyArm.insert(qVecOnlyArm.end(),qVec.end() - 7, qVec.end());
                msg.position() = qVecOnlyArm;

    //            std::cout<<"The size of the postion of the msg is "<<msg.position().size()<<std::endl;
    //            for(size_t i=0;i<msg.position().size();i++){
    //                std::cout<<"SendMsg: Joint "<<i<<" Value: "<<msg.position()[i]<<std::endl;
    //            }
                this->ros2Bridge.SendMsg(msg);
            }

            if(this->isReal){
                // TODO

            }

            // set the initial value of the joint
            qInit = qEigen;

//            qInit = physicalRobotPtr->GetJointsAngleEigen();
            std::cout << "q:\n" << std::fixed << std::setprecision(5) << q << std::endl;
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


        int framePeriod = static_cast<int>(1000.0 / this->FPS);
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
