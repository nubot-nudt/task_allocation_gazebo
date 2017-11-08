#ifndef _NUBOT_TASKALLOCATION_H
#define _NUBOT_TASKALLOCATION_H

#include "nubot/core/core.hpp"
#include <nubot_common/Teleop_joy.h>
#include <nubot_common/WorldModelInfo.h>
#include <nubot_common/BallHandle.h>
#include <nubot_common/Shoot.h>
#include <nubot_common/Allocation_info.h>

#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <time.h>

#include <nubot/nubot_allocation/common.hpp>
#include <nubot/nubot_allocation/behaviour.hpp>

#include "nubot/world_model/robot.h"
#include "nubot/world_model/ball.h"
#include "nubot/world_model/teammatesinfo.h"
#include "nubot/nubot_allocation/dribblestate.hpp"

#define OUTERROR ROS_ERROR
#define Task_Allocation

using namespace nubot;

class World_Model_Info
{
public:
    std::vector<Robot>      RobotInfo_;
    std::vector<BallObject> BallInfo_;
    std::vector<DPoint>     Obstacles_;
    std::vector<DPoint>     Opponents_;
    MessageFromCoach  CoachInfo_;                      // 接收到的COACH信息
    int BallInfoState_;                                // 当前看到足球的状态自己、队友或者没有看到足球
    DPoint lastBallPosition_;                          // 上一帧足球所在的位置
    int  NoSeeBallNums_;                               // 连续几帧没有看到足球
    int  AgentID_;                                     // 机器人自身的ID
    int  CurActiveRobotNums_;                          // 当前活跃的机器人数目
    int  PreActiveRobotNums_;                          // 上一帧活跃的机器人数目
    int  CurActiveNotGoalieNums_;                      // 当前除去守门员活跃的机器人书目
    int  PreActiveNotGoalieNums_;                      // 上一帧除去守门员活跃的机器人书目
    bool RegainBallInOurFiled_;                        // 得到球是在自己的半场上，现在必须传球,false可以直接射门；
    DribbleState DribbleState_;                        // 判断带球的状态；

    bool isShootflg;                                   // 是否踢球
    bool IsMoveForStartCommand_;
    bool IsOurDribble_;

public:
    World_Model_Info()
    {
        RobotInfo_.resize(OUR_TEAM);
        BallInfo_.resize(OUR_TEAM);
        AgentID_ = 0;
        CurActiveRobotNums_ = 0;
        PreActiveRobotNums_ = 0;
        PreActiveNotGoalieNums_ = 0;
        CurActiveNotGoalieNums_ = 0;
        BallInfoState_ = NOTSEEBALL;
        RegainBallInOurFiled_ = false;
        Obstacles_.reserve(25);
        isShootflg=false;
        CoachInfo_.MatchMode = STOPROBOT;
        CoachInfo_.MatchType = Static_Game;
        IsMoveForStartCommand_ =false;
        lastBallPosition_ = DPoint(-650,0);
        NoSeeBallNums_ = 0;

    }
    /** 计算场地上活跃的机器人数目 */
    void caculateActiveRobots()
    {
        PreActiveRobotNums_     = CurActiveRobotNums_;
        PreActiveNotGoalieNums_ = CurActiveNotGoalieNums_;
        CurActiveNotGoalieNums_ = CurActiveRobotNums_ = 0;
        for(std::size_t i = 1 ; i < OUR_TEAM ; i++)
            CurActiveNotGoalieNums_ += RobotInfo_[i].isValid();
        CurActiveRobotNums_ = CurActiveNotGoalieNums_ + RobotInfo_[0].isValid();
    }


    /**  函数的功能给是判断机器人是否带上足球 */
    void checkDribble(const bool & ball_holding)
    {
        DPoint robot_pos = RobotInfo_[AgentID_-1].getLocation();
        DPoint ball_pos = BallInfo_[AgentID_-1].getGlobalLocation();
        DPoint ball2robot= DPoint(BallInfo_[AgentID_-1].getRealLocation());

        double dis2b = robot_pos.distance(ball_pos);
        bool dribblecheck = dis2b <= ConstDribbleDisFirst ? ball_holding : false;
        DribbleState_.update(dribblecheck,robot_pos,ball2robot);
        RobotInfo_[AgentID_-1].setDribbleState(dribblecheck);
    }
};

//关于任务分配的类
class Taskallocation
{
public:
    ros::Subscriber  worldmodelinfo_sub_;
    ros::Publisher   motor_cmd_pub_;
    ros::Publisher   allocation_info_pub_;
    ros::Timer       control_timer_;

    ros::ServiceClient ballhandle_client_;
    ros::ServiceClient shoot_client_;
    boost::shared_ptr<ros::NodeHandle> nh_;
    nubot_common::BallHandle  ballhandle;
    nubot_common::Shoot       shoot;
public:
    World_Model_Info worldmodel_info_;

    DPoint2s Passer_pos;                              //the init_pos of robot
    DPoint2s Receiver1_pos;
    DPoint2s Receiver2_pos;

    bool isReceiver_location;
    bool isPasser_location;
    bool isCooperation_Ready;
    bool isCooperation_done;
    bool isReceiver_receive;
    bool isNotcatch_ball;
    bool isfirst_random;
    bool isfirst_static;
    bool isfirst_dynamic;

    bool pass_which;
    bool receive_not;

    int passID;
    int receive_assistID;
    int receive_defenseID;
    float P_a_A;            //p(a|A)
    float P_c_A;            //p(c|A)
    float P_a_D;            //p(a|D)
    float P_c_D;            //p(c|D)
    float P_a;              //p(a)
    float P_c;              //p(c)

    Behaviour *m_behaviour_;

    float costFun_1[2][8];                            //cost_function
    float costFun_2[2][8];

public:
    Taskallocation(int argc,char **argv);
    ~Taskallocation();

    void update_world_model_info(const nubot_common::WorldModelInfo & _world_msg);
    void loopControl(const ros::TimerEvent& event);
    void setEthercatCommond();
    void pubAllocation_info();
    void stopRobot();
    void processPasser();
    void processReceiver();
    void calculateP();
    bool randomChoice();
    bool passWhich();                                   //1 pass forward, 0 pass back
    bool receiveORnot();                                //1 receive, 0 not receive
};

#endif // _NUBOT_TASKALLOCATION_H
