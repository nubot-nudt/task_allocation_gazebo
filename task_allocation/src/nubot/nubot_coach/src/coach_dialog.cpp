#include "coach_dialog.h"

Dialog::Dialog(nubot::Robot2coach_info & robot2coach, nubot::MessageFromCoach & coach2robot, nubot::Allocation_info &allocation, nubot::Obstacles_pos &obs_pos, QWidget *parent) :
    QDialog(parent)
{
    robot2coach_info_= & robot2coach;
    coach2robot_info_= & coach2robot;
    allocation_info_ = & allocation;
    obstacles_pos_   = & obs_pos;

    ui=new Ui::Dialog;
    ui->setupUi(this);
    R_POS_Show<<ui->position2_show<<ui->position3_show<<ui->position4_show;
    R_TYPE_Show<<ui->type2_show<<ui->type3_show<<ui->type4_show;
    R_ACTION_Show<<ui->action2_show<<ui->action3_show<<ui->action4_show;

    QTimer *timer=new QTimer(this);
    connect(timer,SIGNAL(timeout()),this,SLOT(timerUpdate()));        //timeupdate

    coach2robot_info_->MatchMode=STOPROBOT;                           //init coach info
    coach2robot_info_->MatchType=Static_Game;
    coach2robot_info_->Passer_pos=nubot::DPoint2s(0,0);
    coach2robot_info_->Receiver1_pos=nubot::DPoint2s(450,0);
    coach2robot_info_->Receiver2_pos=nubot::DPoint2s(-450,0);

    this->setFixedSize(370,700);                                      //fix windows size
    timer->start(100);

    ui->type3_show->setText("Active");
    ui->type4_show->setText("Defense");
    ui->obs_pos_show1->setText("(0, 0)");
    ui->obs_pos_show2->setText("(0, 0)");

    pass_ID=2;
    assist_ID=3;
    defense_ID=4;
    is_automation=false;
    count=0;
}

Dialog::~Dialog()
{
    delete ui;
}

void Dialog::keyPressEvent(QKeyEvent *event)
{
    if(event->key()==Qt::Key_Space)
        coach2robot_info_->MatchMode=STOPROBOT;
}

void Dialog::timerUpdate()
{
    static std::ofstream allocation_result("/home/nubot8/allocation_result.txt");
    coach2robot_info_->isCooperation_ready=allocation_info_->isPasserLocation[0]&&allocation_info_->isReceiverLocation[1]&&allocation_info_->isReceiverLocation[2];
    is_cooperation_done_last=is_cooperation_done;
    is_cooperation_done=allocation_info_->isCooperation_done[0]||allocation_info_->isCooperation_done[1]||allocation_info_->isCooperation_done[2];
    nubot::DPoint2s pos;
    nubot::DPoint2s vel;
    QString pos_x;
    QString pos_y;
    QString vel_x;
    QString vel_y;

    //update robot_info
    for(int i=0;i<INVOLVE_NUM;i++)
    {
        pos=robot2coach_info_->RobotInfo_[i+1].getLocation();
        pos_x=QString::number(pos.x_);
        pos_y=QString::number(pos.y_);
        R_POS_Show[i]->setText("("+pos_x+", "+pos_y+")");
    }
    //update ball_info
    pos=robot2coach_info_->BallInfo_[1].getGlobalLocation();
    vel=robot2coach_info_->BallInfo_[1].getVelocity();
    pos_x=QString::number(pos.x_);
    pos_y=QString::number(pos.y_);
    vel_x=QString::number(vel.x_);
    vel_y=QString::number(vel.y_);
    ui->ball_pos_show->setText("("+pos_x+", "+pos_y+")");
    ui->ball_vel_show->setText("("+vel_x+", "+vel_y+")");

    //update allocation_info
    if(coach2robot_info_->isCooperation_ready)
    {
        if(allocation_info_->passWhich[0])
        {
            ui->type2_show->setText("Aggressive");
            ui->action2_show->setText("Pass forward");
        }
        else
        {
            ui->type2_show->setText("conservative");
            ui->action2_show->setText("Pass backward");
        }
    }
    else
    {
        ui->type2_show->setText(" ");
        ui->action2_show->setText(" ");
    }
    for(int i=1;i<INVOLVE_NUM;i++)
    {
        if(coach2robot_info_->isCooperation_ready)
        {
            if(allocation_info_->receiveORnot[i])
                R_ACTION_Show[i]->setText("Receive");
            else
                R_ACTION_Show[i]->setText("Not receive");
        }
        else
            R_ACTION_Show[i]->setText(" ");
    }

    if(is_cooperation_done&&!is_cooperation_done_last)
    {
        QString passwhich=QString::number(allocation_info_->passWhich[pass_ID-2]);
        QString receive_assist=QString::number(allocation_info_->receiveORnot[assist_ID-2]);
        QString receive_defense=QString::number(allocation_info_->receiveORnot[defense_ID-2]);
        QString isreceive=QString::number(allocation_info_->isReceiver_receive[assist_ID-2]||allocation_info_->isReceiver_receive[defense_ID-2]);
        //qDebug()<<allocation_info_->isReceiver_receive[assist_ID-2]<<" "<<allocation_info_->isReceiver_receive[defense_ID-2];

        if(coach2robot_info_->MatchType==Static_Game)
        {
            ui->result_show->append("Static:   ("+passwhich+", "+receive_assist+", "+receive_defense+")"+"  "+isreceive);
            allocation_result<<" static: "<<allocation_info_->passWhich[pass_ID-2]<<" "<<allocation_info_->receiveORnot[assist_ID-2]<<
            " "<<allocation_info_->receiveORnot[defense_ID-2]<<" "<<(allocation_info_->isReceiver_receive[assist_ID-2]||allocation_info_->isReceiver_receive[defense_ID-2]);
            if(is_automation)
            {
                coach2robot_info_->MatchType=Dynamic_Game;
                ui->static_game->setChecked(false);
                ui->dynamic_game->setChecked(true);
            }
        }
        else if(coach2robot_info_->MatchType==Dynamic_Game)
        {
            ui->result_show->append("Dynamic: ("+passwhich+", "+receive_assist+", "+receive_defense+")"+"  "+isreceive);
            allocation_result<<" dynamic: "<<allocation_info_->passWhich[pass_ID-2]<<" "<<allocation_info_->receiveORnot[assist_ID-2]<<
            " "<<allocation_info_->receiveORnot[defense_ID-2]<<" "<<(allocation_info_->isReceiver_receive[assist_ID-2]||allocation_info_->isReceiver_receive[defense_ID-2]);
            if(is_automation)
            {
                coach2robot_info_->MatchType=Random_Choice;
                ui->dynamic_game->setChecked(false);
                ui->random_choice->setChecked(true);
            }
        }
        else if(coach2robot_info_->MatchType==Random_Choice)
        {
            ui->result_show->append("Random:  ("+passwhich+", "+receive_assist+", "+receive_defense+")"+"  "+isreceive);
            allocation_result<<" random: "<<allocation_info_->passWhich[pass_ID-2]<<" "<<allocation_info_->receiveORnot[assist_ID-2]<<
            " "<<allocation_info_->receiveORnot[defense_ID-2]<<" "<<(allocation_info_->isReceiver_receive[assist_ID-2]||allocation_info_->isReceiver_receive[defense_ID-2]);

            allocation_result<<" pass_pos: "<<coach2robot_info_->Passer_pos.x_<<", "<<coach2robot_info_->Passer_pos.y_
                             <<" receiver_pos: "<<coach2robot_info_->Receiver1_pos.x_<<", "<<coach2robot_info_->Receiver1_pos.y_
                             <<" "<<coach2robot_info_->Receiver2_pos.x_<<", "<<coach2robot_info_->Receiver2_pos.y_
                             <<" obstcales_pos: "<<obstacles_pos_->obstacle1.x_<<", "<<obstacles_pos_->obstacle1.y_
                             <<" "<<obstacles_pos_->obstacle2.x_<<", "<<obstacles_pos_->obstacle2.y_<<'\n';
            if(is_automation)
            {
                coach2robot_info_->MatchType=Static_Game;
                ui->random_choice->setChecked(false);
                ui->static_game->setChecked(true);
                coach2robot_info_->MatchMode=STOPROBOT;

                qsrand(QTime(0,0,0).secsTo(QTime::currentTime()));
                // start a new game
                ui->random_obs->click();
                sleep(1);
                ui->random_rob2->click();
                sleep(1);
                ui->random_rob3->click();
                sleep(1);
                ui->random_rob4->click();
                count++;
                QString count_string=QString::number(count);
                ui->count_dis->setText(count_string);
                if(count<=1000)
                    coach2robot_info_->MatchMode=STARTROBOT;
            }
        }
    }
}
void Dialog::on_startButton_clicked()
{
    coach2robot_info_->MatchMode=STARTROBOT;
    is_automation=false;
}

void Dialog::on_stopButton_clicked()
{
    coach2robot_info_->MatchMode=STOPROBOT;
}

void Dialog::on_random_rob2_clicked()
{
    short x;
    short y;
    //qsrand(QTime(0,0,0).secsTo(QTime::currentTime()));

    do
    {
        x=qrand()%((FIELD_LENGTH-200)+1)-(FIELD_XLINE1-100);
        y=qrand()%((FIELD_WIDTH -200)+1)-(FIELD_YLINE1-100);
    }while(coach2robot_info_->Receiver1_pos.distance(nubot::DPoint2s(x,y))<200||
           coach2robot_info_->Receiver2_pos.distance(nubot::DPoint2s(x,y))<200);
    coach2robot_info_->Passer_pos=nubot::DPoint2s(x,y);
}

void Dialog::on_random_rob3_clicked()
{
    short x;
    short y;
    //qsrand(QTime(0,0,0).secsTo(QTime::currentTime()));

    do
    {
        x=qrand()%((FIELD_LENGTH-200)+1)-(FIELD_XLINE1-100);
        y=qrand()%((FIELD_WIDTH -200)+1)-(FIELD_YLINE1-100);
    }while(coach2robot_info_->Passer_pos.distance(nubot::DPoint2s(x,y))<200);
    coach2robot_info_->Receiver1_pos=nubot::DPoint2s(x,y);
    if(coach2robot_info_->Receiver2_pos.x_*x>0)
    {
        do
        {
            x=qrand()%((FIELD_LENGTH-200)+1)-(FIELD_XLINE1-100);
            y=qrand()%((FIELD_WIDTH -200)+1)-(FIELD_YLINE1-100);
        }while(coach2robot_info_->Passer_pos.distance(nubot::DPoint2s(x,y))<200);
        if(coach2robot_info_->Receiver1_pos.x_*x>0)
            x=-x;
        coach2robot_info_->Receiver2_pos=nubot::DPoint2s(x,y);
    }

    if(coach2robot_info_->Receiver1_pos.x_>0)
    {
        ui->type3_show->setText("Assist");
        ui->type4_show->setText("Defense");
        assist_ID=3;
        defense_ID=4;
    }
    else
    {
        ui->type3_show->setText("Defense");
        ui->type4_show->setText("Assist");
        assist_ID=4;
        defense_ID=3;
    }
}

void Dialog::on_random_rob4_clicked()
{
    short x;
    short y;
    //qsrand(QTime(0,0,0).secsTo(QTime::currentTime()));

    do
    {
        x=qrand()%((FIELD_LENGTH-200)+1)-(FIELD_XLINE1-100);
        y=qrand()%((FIELD_WIDTH -200)+1)-(FIELD_YLINE1-100);
    }while(coach2robot_info_->Passer_pos.distance(nubot::DPoint2s(x,y))<200);
    coach2robot_info_->Receiver2_pos=nubot::DPoint2s(x,y);

    if(coach2robot_info_->Receiver1_pos.x_*x>0)
    {
        do
        {
            x=qrand()%((FIELD_LENGTH-200)+1)-(FIELD_XLINE1-100);
            y=qrand()%((FIELD_WIDTH -200)+1)-(FIELD_YLINE1-100);
        }while(coach2robot_info_->Passer_pos.distance(nubot::DPoint2s(x,y))<200);
        if(coach2robot_info_->Receiver2_pos.x_*x>0)
            x=-x;
        coach2robot_info_->Receiver1_pos=nubot::DPoint2s(x,y);
    }

    if(coach2robot_info_->Receiver1_pos.x_>0)
    {
        ui->type3_show->setText("Assist");
        ui->type4_show->setText("Defense");
        assist_ID=3;
        defense_ID=4;
    }
    else
    {
        ui->type3_show->setText("Defense");
        ui->type4_show->setText("Assist");
        assist_ID=4;
        defense_ID=3;
    }
}

void Dialog::on_show_more_clicked()
{
    if(this->size().width()==370)
        this->setFixedSize(560,700);
    else
        this->setFixedSize(370,700);
}

void Dialog::on_random_obs_clicked()
{
    short x;
    short y;

    QString pos_x;
    QString pos_y;

    //obstcale1
    //qsrand(QTime(0,0,0).secsTo(QTime::currentTime()));
    x=qrand()%((FIELD_LENGTH-200)+1)-(FIELD_XLINE1-100);
    y=qrand()%((FIELD_WIDTH -200)+1)-(FIELD_YLINE1-100);
    pos_x=QString::number(x);
    pos_y=QString::number(y);
    ui->obs_pos_show1->setText("("+pos_x+", "+pos_y+")");
    obstacles_pos_->obstacle1=nubot::DPoint2s(x,y);

    //obstcale2
    do
    {
        x=qrand()%((FIELD_LENGTH-200)+1)-(FIELD_XLINE1-100);
        y=qrand()%((FIELD_WIDTH -200)+1)-(FIELD_YLINE1-100);
    }while(obstacles_pos_->obstacle1.distance(nubot::DPoint2s(x,y))<400);
    pos_x=QString::number(x);
    pos_y=QString::number(y);
    ui->obs_pos_show2->setText("("+pos_x+", "+pos_y+")");
    obstacles_pos_->obstacle2=nubot::DPoint2s(x,y);
}

void Dialog::on_stepButton_clicked()
{
    coach2robot_info_->MatchMode=STARTROBOT;
    is_automation=true;
}

void Dialog::on_static_game_clicked()
{
    coach2robot_info_->MatchType=Static_Game;
}

void Dialog::on_dynamic_game_clicked()
{
    coach2robot_info_->MatchType=Dynamic_Game;
}

void Dialog::on_random_choice_clicked()
{
    coach2robot_info_->MatchType=Random_Choice;
}
