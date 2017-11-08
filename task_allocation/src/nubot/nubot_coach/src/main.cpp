#include "coach_dialog.h"
#include <QIcon>
#include <QDebug>
#include <robot2coach.h>
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QString myDir=QCoreApplication::applicationDirPath();
    QDir::setCurrent(myDir);                                                     //改变当前目录到程序目录
    a.setWindowIcon(QIcon(":/app.png"));     //改变应用程序图标

    /** 初始化ROS */
    ros::init(argc,argv,"nubot_coach_node");
    ros::Time::init();
    ROS_INFO("start coach process");

    //ros::NodeHandle node;
    nubot::Robot2coach robot2coach(argv);
    Dialog w(robot2coach.robot2coach_info, robot2coach.coach2robot_info, robot2coach.allocation_info, robot2coach.obs_positions);
    w.show();

    robot2coach.start();
    return a.exec();
}
