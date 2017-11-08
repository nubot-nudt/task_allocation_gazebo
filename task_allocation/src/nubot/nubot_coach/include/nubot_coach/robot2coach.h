#ifndef ROBOT2COACH_H
#define ROBOT2COACH_H

#include "nubot/core/core.hpp"
#include <qdebug.h>
#include <nubot_common/WorldModelInfo.h>
#include <nubot_common/CoachInfo.h>
#include <nubot_common/Allocation_info.h>
#include <nubot_common/Pos2d.h>
#include <nubot/world_model/world_model.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <QtCore>

using namespace std;
namespace nubot {

class Robot2coach_info
{
public:
    vector<Robot>            RobotInfo_;       //机器人的信息
    vector<BallObject>       BallInfo_;        //球的信息
    vector<vector<DPoint2s>> Obstacles_;       //单个机器人识别障碍物信息
    vector<DPoint2s>         Opponents_;       //多个机器人障碍物融合信息
    char Pass_id_;              //传球机器人ID
    char Catch_id_;             //接球机器人ID
    bool isPass_valid_;

    Robot2coach_info()
    {
        RobotInfo_.resize(OUR_TEAM);
        BallInfo_.resize(OUR_TEAM);
        Obstacles_ = std::vector <std::vector<DPoint2s> > (OUR_TEAM,std::vector<DPoint2s>(MAX_OBSNUMBER_CONST));
        Opponents_.reserve(MAX_OBSNUMBER_CONST *2);
    }
};

struct Allocation_info
{
    bool isReceiverLocation[3];
    bool isPasserLocation[3];
    bool passWhich[3];
    bool receiveORnot[3];
    bool isCooperation_done[3];
    bool isReceiver_receive[3];
    bool isNotcatch_ball[3];
};

struct Obstacles_pos
{
    DPoint2s obstacle1;
    DPoint2s obstacle2;
};

class Robot2coach:public QThread
{
public:
    ros::Subscriber  robot2coachinfo_sub_;
    ros::Subscriber  passer_allocation_sub_;
    ros::Subscriber  receiver1_allocation_sub_;
    ros::Subscriber  receiver2_allocation_sub_;
    ros::Publisher   coach2robotinfo_pub_;
    ros::Publisher   coach2obs1pos_pub_;
    ros::Publisher   coach2obs2pos_pub_;
    ros::Timer       coachinfo_publish_timer_;
    nubot_common::CoachInfo coachinfo_publish_info_;
    Robot2coach_info robot2coach_info;
    MessageFromCoach coach2robot_info;
    Allocation_info  allocation_info;
    Obstacles_pos    obs_positions;
    DPoint2s         obs_lastpos;

public:
    Robot2coach(char *argv[]);
    void run();
    void update_info(const nubot_common::WorldModelInfo & _world_msg);
    void update_passerinfo(const nubot_common::Allocation_info & _passer_msg);
    void update_receiver1info(const nubot_common::Allocation_info & _receiver_msg);
    void update_receiver2info(const nubot_common::Allocation_info & _receiver_msg);
    void publish(const ros::TimerEvent &);
    void operator ()();
    ~Robot2coach();
};

}

#endif // ROBOT2COACH_H
