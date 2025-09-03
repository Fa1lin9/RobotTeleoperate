#include "CrpRobotBasicControlGUI.h"
#include "./ui_CrpRobotBasicControlGUI.h"

CrpRobotBasicControlGUI::CrpRobotBasicControlGUI(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::CrpRobotBasicControlGUI)
{
    ui->setupUi(this);
    // Initialize the robot
    PhysicalRobot::config config = {
        .type = PhysicalRobotType::CrpRobot,
    };
    this->physicalRobotPtr = PhysicalRobot::GetPtr(config);
}

CrpRobotBasicControlGUI::~CrpRobotBasicControlGUI()
{
    delete ui;
}

void CrpRobotBasicControlGUI::printMessage(const std::string &message){
    ui->messageTextEdit->append(QString::fromStdString(message));
    std::cout<<message<<std::endl;
}

void CrpRobotBasicControlGUI::CheckConnection(){
    if(!this->physicalRobotPtr->isConnect()){
        printMessage(" [Connect] Please connect to the robot first ! ");
        return;
    }
}

void CrpRobotBasicControlGUI::UpdateConfig(){
    this->crpRobotConfig.useLeftArm = ui->leftArmCheckBox->isChecked();
    this->crpRobotConfig.useRightArm = ui->rightArmCheckBox->isChecked();
    this->crpRobotConfig.useHead = ui->headCheckBox->isChecked();
    this->crpRobotConfig.useWaist = ui->waistCheckBox->isChecked();

    std::vector<std::string> enabledParts;

    if(this->crpRobotConfig.useLeftArm){
        enabledParts.push_back("Left Arm");
    }
    if(this->crpRobotConfig.useRightArm){
        enabledParts.push_back("Right Arm");
    }
    if(this->crpRobotConfig.useHead){
        enabledParts.push_back("Head");
    }
    if(this->crpRobotConfig.useWaist){
        enabledParts.push_back("Waist");
    }

    if(!enabledParts.empty()){
        std::string msg = " [Config] Use ";
        for(size_t i = 0; i < enabledParts.size(); ++i){
            msg += enabledParts[i];
            if(i < enabledParts.size() - 1){
                msg += ", ";
            }
        }
        this->printMessage(msg);
    }

}

void CrpRobotBasicControlGUI::on_socketConnectPushButton_clicked()
{
    //TODO
    printMessage(" [TODO]Socket Connect ");

}


void CrpRobotBasicControlGUI::on_directConnectPushButton_clicked()
{
    // check connection
    if(this->physicalRobotPtr->isConnect()){
        printMessage(" [Connect] You have alreadly connected to the robot ! ");
        return;
    }

    this->physicalRobotPtr->Connect();
    if(!this->physicalRobotPtr->isConnect()){
        printMessage(" [Connect] Can't connect to the robot ! ");
    }else{
        printMessage(" [Connect] Sucessfully connect to the robot ! ");
    }
}


void CrpRobotBasicControlGUI::on_disconnectPushButton_clicked()
{
    // check connection
    if(!this->physicalRobotPtr->isConnect()){
        printMessage(" [Connect] You have alreadly disconnected to the robot ! ");
        return;
    }

    // disconnect
    this->physicalRobotPtr->Disconnect();
    if(!this->physicalRobotPtr->isConnect()){
        printMessage(" [Connect] Sucessfully disconnect to the robot ! ");
    }else{
        printMessage(" [Connect] Can't disconnect to the robot ! ");
    }
}


void CrpRobotBasicControlGUI::on_emergencyStopPushButton_clicked()
{
    CheckConnection();

    // TODO
}


void CrpRobotBasicControlGUI::on_getJointsValuePushButton_clicked()
{
    CheckConnection();

    // TODO
}


void CrpRobotBasicControlGUI::on_getJointsPosePushButton_clicked()
{
    CheckConnection();

    // TODO
}


void CrpRobotBasicControlGUI::on_leftArmCheckBox_stateChanged(int arg1)
{
    this->UpdateConfig();
}


void CrpRobotBasicControlGUI::on_rightArmCheckBox_stateChanged(int arg1)
{
    this->UpdateConfig();
}


void CrpRobotBasicControlGUI::on_headCheckBox_stateChanged(int arg1)
{
    this->UpdateConfig();
}


void CrpRobotBasicControlGUI::on_waistCheckBox_stateChanged(int arg1)
{
    this->UpdateConfig();
}


void CrpRobotBasicControlGUI::on_moveJPushButton_clicked()
{
    CheckConnection();

    // TODO
    std::vector<double> leftArmTarget(7);
    std::vector<double> rightArmTarget(7);

    std::array<QLineEdit*, 7> leftArmLineEdits = {
        ui->leftArmJ1LineEdit,
        ui->leftArmJ2LineEdit,
        ui->leftArmJ3LineEdit,
        ui->leftArmJ4LineEdit,
        ui->leftArmJ5LineEdit,
        ui->leftArmJ6LineEdit
    };

    std::array<QLineEdit*, 7> rightArmLineEdits = {
        ui->rightArmJ1LineEdit,
        ui->rightArmJ2LineEdit,
        ui->rightArmJ3LineEdit,
        ui->rightArmJ4LineEdit,
        ui->rightArmJ5LineEdit,
        ui->rightArmJ6LineEdit
    };

    bool ok = true;
    for (size_t i = 0; i < leftArmLineEdits.size(); ++i) {
        double val = leftArmLineEdits[i]->text().toDouble(&ok);
        if (!ok) {
            std::string message =
                    " [MoveJ] Invalid input at left arm joint " + std::to_string(i+1);
            this->printMessage(message);
            val = 0.0;
        }
        leftArmTarget[i] = val;
    }

    for (size_t i = 0; i < rightArmLineEdits.size(); ++i) {
        double val = rightArmLineEdits[i]->text().toDouble(&ok);
        if (!ok) {
            std::string message =
                    " [MoveJ] Invalid input at right arm joint " + std::to_string(i+1);
            this->printMessage(message);
            val = 0.0;
        }
        leftArmTarget[i] = val;

    }

}


void CrpRobotBasicControlGUI::on_moveLPushButton_clicked()
{
    CheckConnection();

    // TODO
}


void CrpRobotBasicControlGUI::on_clearMessagePushButton_clicked()
{
    ui->messageTextEdit->setText("");
}


void CrpRobotBasicControlGUI::on_backToZeroPushButton_clicked()
{
    CheckConnection();

    // TODO
}

