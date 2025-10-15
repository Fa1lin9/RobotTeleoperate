#include <CrpRobotIKSolver/CrpRobotIKSolver.hpp>

CrpRobotIKSolver::CrpRobotIKSolver(const IKSolver::BasicConfig &config_)
    :maxIteration(config_.maxIteration),
     relativeTol(config_.relativeTol)
{
    LOG_FUNCTION;
    // Init Model
    pinocchio::urdf::buildModel(
                this->modelPath,
                this->robotModel);

    robotModelAD = robotModel.cast<casadi::SX>();

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

    if(config_.baseOffset.size() != 1){
        std::string error = " As for CrpRobotIKSolver,the size of the baseOffset should be 1! ";
        throw std::length_error(error);
    }else{
        this->baseOffset = config_.baseOffset[0];
        this->baseOffsetAD = this->baseOffset.cast<casadi::SX>();
    }

    // Init
    // Initialize qNeutral
    this->Initialize();

    std::cout<<" Init Sucessfully! "<<std::endl;
}

CrpRobotIKSolver::~CrpRobotIKSolver()
{

}

boost::optional<Eigen::VectorXd> CrpRobotIKSolver::Solve(
        const std::vector<Eigen::Matrix4d>& targetPose,
        const Eigen::VectorXd& qInit,
        bool verbose)
{
    // check the target pose
    for(size_t i=0;i<targetPose.size();i++){
        if(!this->isPoseMatrix(targetPose[i])){
            std::cout<<"targetPose is not a pose matrix"<<std::endl;
            return boost::none;
        }
    }

    this->InitializeAD(targetPose,qInit);
    nlopt::opt opt;

    double (*ObjectWrapper)(const std::vector<double>& x,std::vector<double>& grad,void *data);

    // update casadi variable
//    this->qInit = qInit.cast<casadi::SX>();
//    for(size_t i=0;i<this->targetPose.size();i++){
//        this->targetPose[i] = targetPose[i].cast<casadi::SX>();
//    }

    bool useGrad = 1;
    if(useGrad){
        opt = nlopt::opt(nlopt::GD_STOGO , qInit.size());
        ObjectWrapper = [](const std::vector<double>& x,std::vector<double>& grad,void *data)->double{
            Eigen::Map<const Eigen::VectorXd> q(x.data(),x.size());
            CrpRobotData *robotData = static_cast<CrpRobotData*>(data);
            IKSolver::CrpRobotConfig config = {
                .q = q,
                .qInit= robotData->qInit,
                .targetPose = robotData->targetPose,
            };

            casadi::DM qVar = casadi::DM(x);
            grad.resize(robotData->solver->dofTotal);
            std::vector<casadi::DM> output = robotData->solver->mainFunc({qVar});

            double costFunc = double(output[0]);
            casadi::DM gradFunc = output[1];
            std::transform(
                        gradFunc->begin(), gradFunc->end(), grad.begin(),
                        [](const auto& v){ return double(v); }
            );

            return costFunc;
        };
    }else{
        opt = nlopt::opt(nlopt::GN_DIRECT_L , qInit.size());
        ObjectWrapper = [](const std::vector<double>& x,std::vector<double>& grad,void *data)->double{
            Eigen::Map<const Eigen::VectorXd> q(x.data(),x.size());
            CrpRobotData *robotData = static_cast<CrpRobotData*>(data);
            IKSolver::CrpRobotConfig config = {
                .q = q,
                .qInit= robotData->qInit,
                .targetPose = robotData->targetPose,
            };

            return robotData->solver->CostFunc(config);
        };
    }

    // set limitation to joint5
//    this->totalBoundsLower[8] = 0;
//    this->totalBoundsUpper[8] = 0;
//    this->totalBoundsLower[18] = 0;
//    this->totalBoundsUpper[18] = 0;
    // set limitation to joint6
    this->totalBoundsLower[9] = 0;
    this->totalBoundsUpper[9] = 0;
    this->totalBoundsLower[19] = 0;
    this->totalBoundsUpper[19] = 0;
    // set limitation to joint7
    this->totalBoundsLower[10] = 0;
    this->totalBoundsUpper[10] = 0;
    this->totalBoundsLower[20] = 0;
    this->totalBoundsUpper[20] = 0;

    // set bounds
    opt.set_lower_bounds(this->totalBoundsLower);
    opt.set_upper_bounds(this->totalBoundsUpper);

    opt.set_maxeval(this->maxIteration);
    opt.set_xtol_rel(this->relativeTol);

    CrpRobotData robotData = {
        .solver = this,
        .qInit = qInit,
        .targetPose = targetPose,
    };

    opt.set_min_objective(ObjectWrapper, &robotData);

    double funcValue;

    std::vector<double> q(this->dofTotal);
//    q = this->qNeutral;

    auto start = std::chrono::high_resolution_clock::now();

    nlopt::result result = opt.optimize(q, funcValue);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << " optimazation 耗时: " << duration.count() << " ms" << std::endl;
    std::cout << " Function Value = " << funcValue << std::endl;

    if(result<0){
        std::string error = " Optimize failed! ";
        throw std::logic_error(error);

    }

    Eigen::Map<Eigen::VectorXd> qEigen(q.data(),q.size());

    if(verbose){
        // check result
        std::cout<<"------------ Solver Result ------------"<<std::endl;
        std::cout << " Joint Value =\n" << qEigen << std::endl;
        std::cout << " Function Value = " << funcValue << std::endl;
        std::cout<<" Left Arm Translation: \n"<<Forward(qEigen)[0].translation()<<std::endl;
        std::cout<<" Left Arm Rotation: \n"<<Forward(qEigen)[0].rotation()<<std::endl;
        std::cout<<" Right Arm Translation: \n"<<Forward(qEigen)[1].translation()<<std::endl;
        std::cout<<" Rigit Arm Rotation: \n"<<Forward(qEigen)[1].rotation()<<std::endl;
        std::cout<<"------------ Solver Result ------------"<<std::endl;
    }

    return boost::optional<Eigen::VectorXd>(qEigen);
}

void CrpRobotIKSolver::Info(){
    LOG_FUNCTION;

    std::cout << " ----------------------------------------------------- "<< std::endl;

    // 注意 robotModel.joints包含了全部关节，即包含了初始关节
    // 而整个机器人的自由度是指其可动的关节
    // 所以打印的时候我们可以看到总关节是22，但是DOF确实21
    // 因此，在我们打印相关关节的上下界的时候要注意索引的匹配

//    for(pinocchio::JointIndex i=1; i<robotModel.njoints; ++i){
//        std::cout << "Joint " << i << ": " << robotModel.names[i]
//                  << ", type: " << robotModel.joints[i].shortname()
//                  << ", parent: " << robotModel.parents[i] << std::endl;


//        std::cout << " lower limit: " << robotModel.lowerPositionLimit[i-1]
//                  << ", upper limit: " << robotModel.upperPositionLimit[i-1]<<std::endl;
//        std::cout<<std::endl;

//    }

    std::cout << " ----------------------------------------------------- "<< std::endl;

//    std::cout << "Size of joints upper limitation: " << robotModel.upperPositionLimit.rows() << std::endl;
    std::cout << "Number of joints: " << robotModel.njoints << std::endl;
    std::cout << "Number of DOFs: " << robotModel.nv << std::endl;
    std::cout << "Number of frames: " << robotModel.nframes << std::endl;
//    for(size_t i =0;i<robotModel.nframes;i++){
//        std::cout<<" Frame "<<i<<" : "<<robotModel.frames[i]<<std::endl;
//    }
    // universe 1
    // BASE_S 1
    // JOINT and its JOINT_S 21 * 2 = 42
    // Total 44
    std::cout << "Number of q: " << robotModel.nq << std::endl;

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

//    // 打印平移向量
//    std::cout << "leftArmPose position: " << leftArmPose.translation().transpose() << std::endl;
//    std::cout << "rightArmPose position: "  << rightArmPose.translation().transpose() << std::endl;

//    // 打印旋转矩阵
//    std::cout << "leftArmPose rotation:\n" << leftArmPose.rotation() << std::endl;
//    std::cout << "rightArmPose rotation:\n"  << rightArmPose.rotation() << std::endl;

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
//    std::cout << "耗时: " << duration.count() << " ms" << std::endl;
//    Eigen::VectorXd testQ = Eigen::VectorXd::Zero(dofTotal);

//    testQ.segment(4, dofArm) << -2.09, -0.5236, 0.698, -1, 2.09, 0, -0.733;
//    testQ.segment(4, dofArm) << -0.2, -1.22, -2.09, -1.37, 2.79, 1.24, -0.0;

//    std::cout << " translations of left arm: \n"<<Forward(testQ)[0].translation()<<std::endl;
//    std::cout << " rotation of left arm: \n"<<Forward(testQ)[0].rotation()<<std::endl;
}

std::vector<pinocchio::SE3> CrpRobotIKSolver::Forward(const Eigen::VectorXd& q){
//    LOG_FUNCTION;
    if(q.size() != this->robotModel.nq){
        std::string error = " The size of the q should be this->robotModel.nq! ";
        throw std::length_error(error);
    }

    // updata data to better get position
    pinocchio::Data data=pinocchio::Data(robotModel);
    pinocchio::forwardKinematics(robotModel,data,q);
    pinocchio::updateFramePlacements(robotModel,data);

    // base pose
    pinocchio::SE3 BasePoseOffset = pinocchio::SE3(
                this->baseOffset.block<3,3>(0,0),
                this->baseOffset.block<3,1>(0,3));
    pinocchio::SE3 basePose = data.oMf[this->baseIndex];

//    std::cout<<"BasePose Translation: "<<basePose.translation()<<std::endl;
//    std::cout<<"BasePose Rotation: "<<basePose.rotation()<<std::endl;

    basePose = BasePoseOffset * basePose;

    // left arm
    pinocchio::SE3 leftArmEndPose = data.oMf[this->leftArmEndIndex];
    pinocchio::SE3 leftArmPose = basePose.inverse() * leftArmEndPose;


    // right arm
    pinocchio::SE3 rightArmEndPose = data.oMf[this->rightArmEndIndex];
    pinocchio::SE3 rightArmPose = basePose.inverse() * rightArmEndPose;

    return std::vector<pinocchio::SE3>{leftArmPose, rightArmPose};
}

bool CrpRobotIKSolver::isPoseMatrix(const Eigen::Matrix4d &mat,
                                    const double& eps){

    Eigen::Matrix3d rotation = mat.block<3,3>(0,0);
    if(!(rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity() , eps)){
        std::cout<<"rotation.transpose() * rotation: \n"<<rotation.transpose() * rotation<<std::endl;
        return false;
    }

    if(std::abs(mat.determinant()-1) > eps){
        std::cout<<"mat.determinant()-1: "<<mat.determinant()-1<<std::endl;
        return false;
    }

    if(mat(3,0) != 0.0 || mat(3,1) != 0.0 || mat(3,2) != 0.0 || mat(3,3) != 1.0){
        return false;
    }

    return true;
}

double CrpRobotIKSolver::CostFunc(const IKSolver::CrpRobotConfig& config_){
//    LOG_FUNCTION;
    std::vector<pinocchio::SE3> currentPose = this->Forward(config_.q);

    // translation error
//    std::cout<<" Translation Error "<<std::endl;
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
//    std::cout<<" rotation Error "<<std::endl;
    Eigen::Matrix3d leftArmError
            = config_.targetPose[0].block<3,3>(0,0) * currentPose[0].rotation().inverse();
    Eigen::Matrix3d rightArmError
            = config_.targetPose[1].block<3,3>(0,0) * currentPose[1].rotation().inverse();

    Eigen::VectorXd rotaErrorVec(6);
    rotaErrorVec << pinocchio::log3(leftArmError), pinocchio::log3(rightArmError);
    double rotaError = rotaErrorVec.norm();

    // smoothing error
//    std::cout<<" Smoothing Error "<<std::endl;
    Eigen::VectorXd smoothErrorVec = config_.q - config_.qInit;
    this->NormalizeAngle(smoothErrorVec);
    double smoothError = smoothErrorVec.norm();

    // regularization
//    std::cout<<" Regularization "<<std::endl;
    Eigen::VectorXd reguVec;
    reguVec =
            config_.q - Eigen::VectorXd::Map(this->qNeutral.data(), this->qNeutral.size());
    double reguError = reguVec.norm();

    // weight
    double wTrans = 50.0;
    double wRota = 0.5;
    double wSmooth = 0.1;
    double wRegu = 0.02;

    double error = wTrans * transError + wRota * rotaError + wSmooth * smoothError + wRegu * reguError;
    return error;
}

Eigen::VectorXd CrpRobotIKSolver::GetGradient(const IKSolver::CrpRobotConfig& config_){
    return GradFunc(config_);
}

Eigen::VectorXd CrpRobotIKSolver::GradFunc(const IKSolver::CrpRobotConfig& config_){
    double e = 1e-6;
    Eigen::VectorXd grad(config_.q.size());
    IKSolver::CrpRobotConfig temp = config_;

    for(size_t i = 0;i<config_.q.size();i++){
        double originalValue = config_.q[i];
        // plus
        temp.q[i] = originalValue + e;
        double fPlus = this->CostFunc(temp);

        // minus
        temp.q[i] = originalValue - e;
        double fMinus = this->CostFunc(temp);

        grad[i] = (fPlus - fMinus) / (2*e);

        temp.q[i] = originalValue;
    }

    return grad;
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

void CrpRobotIKSolver::Initialize(){
//    LOG_FUNCTION;
    double min,max;
    for(size_t i = 0;i<this->dofArm;i++){
        // left arm
        min = this->robotModel.lowerPositionLimit(this->leftArmID[0] + i - 1);
        max = this->robotModel.upperPositionLimit(this->leftArmID[0] + i - 1);
        this->leftArmBoundsLower.push_back(min);
        this->leftArmBoundsUpper.push_back(max);
        this->qLeftArmNeutral.push_back((min + max) / 2);

        // right arm
        min = this->robotModel.lowerPositionLimit(this->rightArmID[0] + i - 1);
        max = this->robotModel.upperPositionLimit(this->rightArmID[0] + i - 1);
        this->rightArmBoundsLower.push_back(min);
        this->rightArmBoundsUpper.push_back(max);
        this->qRightArmNeutral.push_back((min + max) / 2);
    }

    // order: base(1) - waist(3) - left arm(dofArm) - neck(3) - right arm(dofArm)
    qNeutral.clear();
    qNeutral.reserve(this->dofTotal);

    // base
    qNeutral.push_back(0);
    totalBoundsLower.push_back(0);
    totalBoundsUpper.push_back(0);

    // waist
    qNeutral.insert(qNeutral.end(), 3, 0);
    totalBoundsLower.insert(totalBoundsLower.end(), 3, 0);
    totalBoundsUpper.insert(totalBoundsUpper.end(), 3, 0);

    // left arm
    qNeutral.insert(qNeutral.end(), qLeftArmNeutral.begin(), qLeftArmNeutral.end());
    totalBoundsLower.insert(totalBoundsLower.end(), leftArmBoundsLower.begin(), leftArmBoundsLower.end());
    totalBoundsUpper.insert(totalBoundsUpper.end(), leftArmBoundsUpper.begin(), leftArmBoundsUpper.end());

    // neck
    qNeutral.insert(qNeutral.end(), 3, 0);
    totalBoundsLower.insert(totalBoundsLower.end(), 3, 0);
    totalBoundsUpper.insert(totalBoundsUpper.end(), 3, 0);

    // right arm
    qNeutral.insert(qNeutral.end(), qRightArmNeutral.begin(), qRightArmNeutral.end());
    totalBoundsLower.insert(totalBoundsLower.end(), rightArmBoundsLower.begin(), rightArmBoundsLower.end());
    totalBoundsUpper.insert(totalBoundsUpper.end(), rightArmBoundsUpper.begin(), rightArmBoundsUpper.end());

//    std::cout<<" The size of the totalBoundsLower is "<<totalBoundsLower.size()<<std::endl;
//    std::cout<<" The size of the totalBoundsUpper is "<<totalBoundsUpper.size()<<std::endl;
//    std::cout<<" The size of the qNeutral is "<<qNeutral.size()<<std::endl;

}

void CrpRobotIKSolver::InitializeAD(const std::vector<Eigen::Matrix4d>& targetPose_,
                                    const Eigen::VectorXd& qInit_){
    // Variable
    casadi::SX qVar = casadi::SX::sym("qVar",this->dofTotal,1);

    pinocchio::DataTpl<casadi::SX>::ConfigVectorType q =
            pinocchio::DataTpl<casadi::SX>::ConfigVectorType::Zero(this->dofTotal);

    std::vector<Eigen::Matrix<casadi::SX,4,4>> targetPose(targetPose_.size());
    for(size_t i=0;i<targetPose_.size();i++){
        targetPose[i] = targetPose_[i].cast<casadi::SX>();
    }

    Eigen::Matrix<casadi::SX,Eigen::Dynamic,1> qInit = qInit_.cast<casadi::SX>();

    for(int i =0;i<q.size();i++){
        q(i) = qVar(i);
    }

    casadi::SX costFunc = this->CostFuncAD(q,qInit,targetPose);
    casadi::SX gradFunc = gradient(costFunc,qVar);
    this->mainFunc = casadi::Function("mainFunc", {qVar}, {costFunc, gradFunc});
}

casadi::SX CrpRobotIKSolver::CostFuncAD(
            const pinocchio::ModelTpl<casadi::SX>::ConfigVectorType& q,
            const Eigen::Matrix<casadi::SX,Eigen::Dynamic,1>& qInit,
            const std::vector<Eigen::Matrix<casadi::SX,4,4>>& targetPose){
    if(q.size() != this->robotModel.nq){
        std::string error = " The size of the q should be this->robotModel.nq! ";
        throw std::length_error(error);
    }
    if(targetPose.size() != 2){
        std::string error = " The size of the targetPose should be 2! ";
        throw std::length_error(error);
    }

    // updata data to better get position
    pinocchio::DataTpl<casadi::SX> dataAD(robotModelAD);
    forwardKinematics(robotModelAD, dataAD, q);
    updateFramePlacements(robotModelAD, dataAD);

    // extract matrix
    Eigen::Matrix<casadi::SX,4,4> basePose =
            this->baseOffsetAD * dataAD.oMf[this->baseIndex].toHomogeneousMatrix();
    Eigen::Matrix<casadi::SX,4,4> leftArmEndPose = dataAD.oMf[this->leftArmEndIndex].toHomogeneousMatrix();
    Eigen::Matrix<casadi::SX,4,4> rightArmEndPose = dataAD.oMf[this->rightArmEndIndex].toHomogeneousMatrix();

    Eigen::Matrix<casadi::SX,4,4> leftArmPose = basePose.inverse() * leftArmEndPose;
    Eigen::Matrix<casadi::SX,4,4> rightArmPose = basePose.inverse() * rightArmEndPose;

//    Eigen::Matrix<casadi::SX,4,4> leftArmPose = this->BaseOffsetAD * leftArmEndPose;
//    Eigen::Matrix<casadi::SX,4,4> rightArmPose = this->BaseOffsetAD * rightArmEndPose;

    // translation error
//    std::cout<<" Translation Error "<<std::endl;
    Eigen::Matrix<casadi::SX,3,1> currentLeftTrans  = leftArmPose.block<3,1>(0,3);
    Eigen::Matrix<casadi::SX,3,1> currentRightTrans = rightArmPose.block<3,1>(0,3);
    Eigen::Matrix<casadi::SX,3,1> targetLeftTrans   = targetPose[0].block<3,1>(0,3);
    Eigen::Matrix<casadi::SX,3,1> targetRightTrans  = targetPose[1].block<3,1>(0,3);

    Eigen::Matrix<casadi::SX,6,1> currentTrans, targetTrans;
    currentTrans.block<3,1>(0,0) = currentLeftTrans;
    currentTrans.block<3,1>(3,0) = currentRightTrans;

    targetTrans.block<3,1>(0,0) = targetLeftTrans;
    targetTrans.block<3,1>(3,0) = targetRightTrans;

    Eigen::Matrix<casadi::SX,6,1> transErrorVec = currentTrans - targetTrans;
    casadi::SX transError = transErrorVec.norm();

    // rotation error
//    std::cout<<" rotation Error "<<std::endl;
    Eigen::Matrix<casadi::SX,3,3> leftArmError
            = targetPose[0].block<3,3>(0,0) * leftArmPose.block<3,3>(0,0);
    Eigen::Matrix<casadi::SX,3,3> rightArmError
            = targetPose[1].block<3,3>(0,0) * rightArmPose.block<3,3>(0,0);

    Eigen::Matrix<casadi::SX,3,1> leftRotError  = pinocchio::log3(leftArmError).cast<casadi::SX>();
    Eigen::Matrix<casadi::SX,3,1> rightRotError = pinocchio::log3(rightArmError).cast<casadi::SX>();

    Eigen::Matrix<casadi::SX,6,1> rotaErrorVec;
    rotaErrorVec.block<3,1>(0,0) = leftRotError;
    rotaErrorVec.block<3,1>(3,0) = rightRotError;

    casadi::SX rotaError = rotaErrorVec.norm();

    // smoothing error
//    std::cout<<" Smoothing Error "<<std::endl;
    pinocchio::DataTpl<casadi::SX>::ConfigVectorType smoothErrorVec =
            q - qInit.cast<casadi::SX>();
    casadi::SX smoothError = smoothErrorVec.norm();

    // regularization
//    std::cout<<" Regularization "<<std::endl;
    pinocchio::DataTpl<casadi::SX>::ConfigVectorType reguVec =
            q - Eigen::VectorXd::Map(this->qNeutral.data(), this->qNeutral.size()).cast<casadi::SX>();
    casadi::SX reguError = reguVec.norm();

    // weight
    double wTrans = 50.0;
    double wRota = 0.5;
    double wSmooth = 0.1;
    double wRegu = 0.02;
//    double wSmooth = 0;
//    double wRegu = 0;

    casadi::SX error =  casadi::SX(wTrans)  * transError +
                        casadi::SX(wRota)   * rotaError +
                        casadi::SX(wSmooth) * smoothError +
                        casadi::SX(wRegu)   * reguError;
    return error;

}
