#ifndef CRPROBOTBASICCONTROLGUI_H
#define CRPROBOTBASICCONTROLGUI_H

#include <QMainWindow>
#include <PhysicalRobot/PhysicalRobot.hpp>
#include <math.h>

QT_BEGIN_NAMESPACE
namespace Ui { class CrpRobotBasicControlGUI; }
QT_END_NAMESPACE

class CrpRobotBasicControlGUI : public QMainWindow
{
    Q_OBJECT

public:
    CrpRobotBasicControlGUI(QWidget *parent = nullptr);
    ~CrpRobotBasicControlGUI();

private:
    void printMessage(const std::string& message);

    void CheckConnection();

    // while the checkbox's state is changed,then update the configuration
    void UpdateConfig();

private slots:
    void on_socketConnectPushButton_clicked();

    void on_directConnectPushButton_clicked();

    void on_disconnectPushButton_clicked();

    void on_emergencyStopPushButton_clicked();

    void on_getJointsValuePushButton_clicked();

    void on_getJointsPosePushButton_clicked();

    void on_leftArmCheckBox_stateChanged(int arg1);

    void on_rightArmCheckBox_stateChanged(int arg1);

    void on_headCheckBox_stateChanged(int arg1);

    void on_waistCheckBox_stateChanged(int arg1);

    void on_moveJPushButton_clicked();

    void on_moveLPushButton_clicked();

    void on_clearMessagePushButton_clicked();

    void on_backToZeroPushButton_clicked();

    void on_resetLeftArmPushButton_clicked();

    void on_resetRightArmPushButton_clicked();

    void on_backToInitPosePushButton_clicked();

private:
    Ui::CrpRobotBasicControlGUI *ui;

    boost::shared_ptr<PhysicalRobot> physicalRobotPtr;

    PhysicalRobot::CrpRobotConfig crpRobotConfig;

    std::vector<double> ConvertDegrees2Radians(const std::vector<double>& degrees);

    std::vector<double> ConvertRadians2Degrees(const std::vector<double>& radians);

    void NormalizeAngle();

    const double pi = M_PI;


};
#endif // CRPROBOTBASICCONTROLGUI_H
