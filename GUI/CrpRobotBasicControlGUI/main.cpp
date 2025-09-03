#include "CrpRobotBasicControlGUI.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    CrpRobotBasicControlGUI w;
    w.show();
    return a.exec();
}
