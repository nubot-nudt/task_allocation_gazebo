#ifndef __NUBOT_CORE_HPP__
#define __NUBOT_CORE_HPP__

#include <fstream>
#include <string>
#include "Circle.hpp"
#include "Angle.hpp"
#include "DPoint.hpp"
#include "PPoint.hpp"
#include "Line.hpp"

#define SIMULATION

#define WAIT_SECS   7       // 根据规则，需要等待多少秒之后才能动

#ifdef SIMULATION
const double WIDTHRATIO4FIELD = 1;
const double WIDTH_RATIO= 1 ;
const double ConstDribbleDisFirst  = 50;
const double ConstDribbleDisSecond = 40;
const int FIELD_CENTER_RADIUS = 200;
#else
const double WIDTHRATIO4FIELD = 8.0/12.0;//1;//8.0/12.0 ;
const double WIDTH_RATIO= 8.0/12.0;//1;//8.0/12.0 ;
const double ConstDribbleDisFirst  = 31;
const double ConstDribbleDisSecond = 30;
const int FIELD_CENTER_RADIUS = 150;
#endif
const int NOT_DATAUPDATE = 1200; //数据没有更新的阈值，比如通信过程中时间大于300ms为更新数据，默认为失效
const int OUR_TEAM = 5 ;        //自己机器人个数
const int OPP_TEAM = 7 ;        //对方的机器人个数
const int ROLENUM = 7;

/** 一些常用的场地参数 */
#if defined(SIMULATION) || defined(MATCH)
const int FIELD_LENGTH= 1800;    //场地参数，长度
const int FIELD_WIDTH = 1200;     //场地参度，长度
const double MAXDIS_FIELD = sqrt(FIELD_LENGTH*FIELD_LENGTH+FIELD_WIDTH*FIELD_WIDTH); //对角线长度
#else
const int FIELD_LENGTH= 1800;    //场地参数，长度
const int FIELD_WIDTH = 800;//1200;//800;     //场地参度，长度
const double MAXDIS_FIELD = sqrt(FIELD_LENGTH*FIELD_LENGTH+FIELD_WIDTH*FIELD_WIDTH); //对角线长度
#endif

/** 场地上的几条水平和垂直线,，其他的信息都可以通过场地信息得到*/
const int FIELD_XLINE1 = 900;
const int FIELD_XLINE2 = 825;
const int FIELD_XLINE3 = 675;
const int FIELD_XLINE4 = 0;
const int FIELD_XLINE5 = -675;
const int FIELD_XLINE6 = -825;
const int FIELD_XLINE7 = -900;

#if defined(SIMULATION) || defined(MATCH)
const int FIELD_YLINE1 =  600;
const int FIELD_YLINE2 =  325;
const int FIELD_YLINE3 =  175;
const int FIELD_YLINE4 =  -175;
const int FIELD_YLINE5 =  -325;
const int FIELD_YLINE6 =  -600;
#else
const int FIELD_YLINE1 = 400;//600;//400;
const int FIELD_YLINE2 = 217;//325;//217;
const int FIELD_YLINE3 = 117;//175;//117;
const int FIELD_YLINE4 = -117;//-175;//-117;
const int FIELD_YLINE5 = -217;//-325;//-217;
const int FIELD_YLINE6 = -400;//-600;//-400;
#endif

const int FIELD_POST_RADIUS = 80;
const int LOCATIONERROR = 30;
const int ANGLEERROR = 5;                 //dw 6.16

/** 场地边界 **/
enum Border{LEFTBORDER = 0,
            RIGHTBORDER = 1,
            UPBORDER = 2,
            DOWNBORDER = 3};

/** 比赛模式的一些定义，*/
enum Roles{GOALIE = 0 ,
           ACTIVE = 1,
           PASSIVE = 2,
           MIDFIELD = 3,
           ASSISTANT = 4,
           ACIDPASSIVE = 5,
           GAZER = 6 ,
           BLOCK = 7,
           NOROLE = 8,
           CATCHOFPASS = 9,
           PASSOFPASS =10};

enum MatchMode {
                 STOPROBOT  =  0,
                 OUR_KICKOFF = 1,
                 OPP_KICKOFF = 2,
                 OUR_THROWIN = 3,
                 OPP_THROWIN = 4,
                 OUR_PENALTY = 5,
                 OPP_PENALTY = 6,
                 OUR_GOALKICK = 7 ,
                 OPP_GOALKICK = 8,
                 OUR_CORNERKICK = 9,
                 OPP_CORNERKICK = 10,
                 OUR_FREEKICK = 11,
                 OPP_FREEKICK = 12,
                 DROPBALL     = 13,
                 STARTROBOT   = 15,
                 PARKINGROBOT = 25,
                 TEST = 27
               };

enum StrategyTyples {  STRATEGY_ATTACK = 0,
                       STRATEGY_DEFEND = 1,
                       STRATEGY_BALANCE =2,
                       STRATEGY_AUTO = 4,
                       NOSTRATEG =5
};

enum TestMode
{
    Test_Stop=0,
    Move_NoBall_NoAvoid = 10,
    Move_NoBall_Avoid   = 11,
    Move_Ball_NoAvoid   = 12,
    Move_Ball_Avoid     = 13,
    Pass_Ball  =2,
    Catch_Ball =3,
    Shoot_Ball =4,
    Location_test =5,
    Circle_test = 6
};

enum GameMode
{
    Static_Game =0,
    Dynamic_Game=1,
    Random_Choice=2
};

enum Actions
{
    Stucked           =0,
    Penalty           =1,
    CanNotSeeBall     =2,
    SeeNotDribbleBall =3,
    TurnForShoot      =4,                   //环绕球
    TurnForShoot_Robot=5,                   //环绕机器人
    AtShootSituation  =6,
    TurnToPass        =7,
    TurnToPass_Robot  =8,
    StaticPass        =9,
    AvoidObs          =10,
    Catch_Positioned  =11,
    Positioned        =12,
    Positioned_Static =13,
    KickCoop          =14,
    KickCoop_turn     =15,
    CatchBall         =16,
    CatchBall_slow    =17,
    CircleTest        =18,
    MoveWithBall      =19,
    TeleopJoy         =20,
    No_Action         =21,
};

struct obs_info
{
    nubot::PPoint polar_pt;
    nubot::DPoint world_pt;
    double HRZ[4];
    double HRH[4*4];
};



#endif 
