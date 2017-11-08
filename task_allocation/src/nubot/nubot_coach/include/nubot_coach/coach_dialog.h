#ifndef DIALOG_H
#define DIALOG_H

#include <qdialog.h>
#include <qstring.h>
#include <qvector.h>
#include <qevent.h>
#include <qtimer.h>
#include <qdatetime.h>
#include <qdebug.h>
#include <qlist.h>
#include <qlineedit.h>
#include <ui_coach_dialog.h>
#include <robot2coach.h>
#include <qmessagebox.h>

#define INVOLVE_NUM 3

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
    Dialog(nubot::Robot2coach_info & robot2coach, nubot::MessageFromCoach & coach2robot, nubot::Allocation_info & allocation, nubot::Obstacles_pos & obs_pos, QWidget *parent = 0);
    ~Dialog();
    void keyPressEvent(QKeyEvent *event);

    nubot::Robot2coach_info * robot2coach_info_;        //用于存放机器人上传的信息
    nubot::MessageFromCoach * coach2robot_info_;        //用于存放coach下发的信息
    nubot::Allocation_info  * allocation_info_;
    nubot::Obstacles_pos    * obstacles_pos_;

private:
    Ui::Dialog *ui;
    QList<QLineEdit*>R_POS_Show;
    QList<QLineEdit*>R_TYPE_Show;
    QList<QLineEdit*>R_ACTION_Show;

    int assist_ID;
    int defense_ID;
    int pass_ID;
    int count;

    bool is_automation;
    bool is_cooperation_done;
    bool is_cooperation_done_last;

private slots:
    void timerUpdate();                              //定时器槽函数
    void on_startButton_clicked();                   //控制面板的槽函数
    void on_stopButton_clicked();
    void on_random_rob2_clicked();
    void on_random_rob3_clicked();
    void on_random_rob4_clicked();
    void on_show_more_clicked();
    void on_random_obs_clicked();
    void on_stepButton_clicked();
    void on_static_game_clicked();
    void on_dynamic_game_clicked();
    void on_random_choice_clicked();
};


#endif // DIALOG_H
