#include <stdio.h>
#include <signal.h>
#include <time.h>
#include "nubot/world_model/world_model.h"
using namespace nubot;
nubot::World_Model::World_Model(int argc,char** argv)
{
    const char * environment;
    std::string robot_name = argv[1];
    std::string num = robot_name.substr(robot_name.size()-1);
    std::string robot_prefix = robot_name.substr(0,robot_name.size()-1);
    environment = num.c_str();
    ROS_INFO("world_model: robot_name:%s",robot_name.c_str());
    nh = boost::make_shared<ros::NodeHandle>(robot_name);
    std::string info_topic = "/" + robot_prefix + "/receive_from_coach";

    /** coach topic*/
    coach_sub_ = nh->subscribe(info_topic, 1 , &nubot::World_Model::receiveFromCoach, this);

    /** 订阅全向视觉节点topic，所有的机器人*/
    omin_vision_sub_    =  nh->subscribe("omnivision/OmniVisionInfo", 1 , &nubot::World_Model::updateOminivision, this);

    /** 发布更新的世界模型信息，包括感知信息，COACH信息等等 */
    worldmodelinfo_pub_ =  nh->advertise<nubot_common::WorldModelInfo>("worldmodel/worldmodelinfo",10);
    /** 30ms触发一次的定时器 */
    worldmodel_update_timer_ = nh->createTimer(ros::Duration(0.015),&World_Model::update,this);
    teammatesinfo_.resize(OUR_TEAM); //开辟内存空间；
    // 读取机器人标号，并赋值.
    AgentID_ = atoi(environment);
    teammatesinfo_[AgentID_-1].robot_info_.setID(AgentID_); //机器人ID标号设置；
    for(int i = 0; i<OUR_TEAM;i++)
       teammatesinfo_[AgentID_-1].robot_info_.setValid(false);
    obstacles_.setAgentID(AgentID_);
    teammateIDforBallSelected = -1;
    // 接收到的各种topic更新时间；
    omni_update_time_   = ros::Time::now();     // 机器人信息的更新时间，用于计算lifetime判断机器人信息的有效性；
    /** 当前仅仅传输五个机器人信息*/
    world_model_info_.robotinfo.resize(OUR_TEAM);
    receive_coach_count_ = ros::Time::now();;
    coach2robot_.MatchMode = STOPROBOT;
    coach2robot_.MatchType = Static_Game;
}

nubot::World_Model::~World_Model()
{

}

void
nubot::World_Model::receiveFromCoach(const nubot_common::CoachInfo & _coach)
{
    coach2robot_.MatchMode          = _coach.MatchMode ;
    coach2robot_.MatchType          = _coach.MatchType ;
    coach2robot_.Passer_pos.x_      = _coach.Passer_pos.x;
    coach2robot_.Passer_pos.y_      = _coach.Passer_pos.y;
    coach2robot_.Receiver1_pos.x_   = _coach.Receiver1_pos.x;
    coach2robot_.Receiver1_pos.y_   = _coach.Receiver1_pos.y;
    coach2robot_.Receiver2_pos.x_   = _coach.Receiver2_pos.x;
    coach2robot_.Receiver2_pos.y_   = _coach.Receiver2_pos.y;
    coach2robot_.isCooperation_ready= _coach.isCooperation_ready;
}

/**
    * 接收全向视觉节点发布的消息，当前用于所有的机器人；
    * 更新世界模型中robot_info，ball_info_，以及obstacles_；
    * @param [in] omni_info：全向视觉节点发布的消息，包含有机器人定位信息、障碍物信息、足球信息等等
    */
void
nubot::World_Model::updateOminivision(const nubot_common::OminiVisionInfo & omni_info)
{
    omni_update_time_ = omni_info.header.stamp;
    /** 给机器人信息赋值，其中特别要说明的是setValid表示机器人的开关电状态，关闭总开关时false */
    int robot_nums = omni_info.robotinfo.size();
    for(int  i = 0 ; i < robot_nums; i++)
    {
        int AgentId = omni_info.robotinfo[i].AgentID;
        if(AgentId < 1 || AgentId >OUR_TEAM)
            continue;
        Robot & robot_info = teammatesinfo_[AgentId-1].robot_info_;
        robot_info.setID(AgentId);
        robot_info.setVelocity(DPoint2d(omni_info.robotinfo[i].vtrans.x,omni_info.robotinfo[i].vtrans.y));
        robot_info.setLocation(DPoint2d(omni_info.robotinfo[i].pos.x,omni_info.robotinfo[i].pos.y));
        robot_info.setW(omni_info.robotinfo[i].vrot);
        robot_info.setHead(Angle(omni_info.robotinfo[i].heading.theta));
        robot_info.setStuck(omni_info.robotinfo[i].isstuck);
        robot_info.setValid(omni_info.robotinfo[i].isvalid);
    }
    /** 给障碍物信息赋值 */
    std::vector< ObstacleObject > obstacles;
    int length=omni_info.obstacleinfo.pos.size();
    obstacles.reserve(length);
    for(int i = 0 ;i < length ; i++)
    {
        ObstacleObject object_temp;
        DPoint pt(omni_info.obstacleinfo.pos[i].x,omni_info.obstacleinfo.pos[i].y);
        PPoint polar(Angle(omni_info.obstacleinfo.polar_pos[i].angle),
                     omni_info.obstacleinfo.polar_pos[i].radius);
        object_temp.setLocation(pt);
        object_temp.setPolarLocation(polar);
        obstacles.push_back(object_temp);
    }
    obstacles_.setOmniObstacles(obstacles,AgentID_);
    /** 给world_model中的ball_info_类中的omni_ball_变量赋值 */
    BallObject omni_ball;
    omni_ball.setGlobalLocation(DPoint(omni_info.ballinfo.pos.x,omni_info.ballinfo.pos.y));
    /** 表示足球位置是否已知，是否检测到足球 */
    omni_ball.setLocationKnown(omni_info.ballinfo.pos_known);
    omni_ball.setRealLocation(PPoint(Angle(omni_info.ballinfo.real_pos.angle),
                                     omni_info.ballinfo.real_pos.radius));
    omni_ball.setID(AgentID_);
    omni_ball.setValid(true);

    /** 仿真时候足球的velocity信息已知*/
    omni_ball.setVelocity(DPoint2d(omni_info.ballinfo.velocity.x,omni_info.ballinfo.velocity.y));
    omni_ball.setVelocityKnown(omni_info.ballinfo.velocity_known);
    ball_info_.sensor_ball_[OMNI_BALL] = omni_ball;

    /*ROS_INFO("omni: %d  %d  %d  %.2f %.2f %.2f %d ",
             !is_start_again,
             ball_info_.sensor_ball_[OMNI_BALL].isVelocityKnown(),
             ball_info_.sensor_ball_[OMNI_BALL].isLocationKnown(),
             ball_info_.sensor_ball_[OMNI_BALL].getVelocity().x_,
             ball_info_.sensor_ball_[OMNI_BALL].getVelocity().y_,
             ball_info_.sensor_ball_[OMNI_BALL].getRealLocation().radius_,
             ball_info_.omni_ball_record_.size());*/
}
/**
    * 周期更新世界模型（30ms，ROS定时器）
    * 因为接收的各种topic信息是基于消息中断的，世界模型无法直接判断是否数据已经更新
    * 采用定时器更新世界模型的信息，可以判断信息是否有用，足球、障碍物、机器人、队友是否处于正常状态（根据数据更新的时间戳）
    * 同时可以对所有信息进行融合，得到一个最终统一的世界模型
    */
void
nubot::World_Model::update(const ros::TimerEvent & )
{
    static ros::Time time_before = ros::Time::now();
    ros::Duration duration = ros::Time::now() - time_before;
    time_before = ros::Time::now();
    static  int  streaming_cout = 0;

    /** 所有的信息均认为有效，不需要额外的融合措施，仅仅需要将其发布到控制节点和COACH即可*/
    /** 直接填充障碍物信息*/
    std::vector< ObstacleObject> omni_obstacles ;
    obstacles_.getOmniObstacles(omni_obstacles,AgentID_);
    obstacles_.fuse_obs_.clear();
    obstacles_.self_obs_.clear();
    for(int i =0; i <omni_obstacles.size(); i++)
    {
        obstacles_.self_obs_.push_back(omni_obstacles[i].getLocation());
        bool isteammates =false;
        for(int j = 0 ; j< OUR_TEAM; j++)
        {
            nubot::Robot & robot_info = teammatesinfo_[j].robot_info_;
            if(robot_info.isValid())
            {
                double distance = robot_info.getLocation().distance(omni_obstacles[i].getLocation());
                if(distance < 10)
                    isteammates = true;
            }
        }
        if(!isteammates)
            obstacles_.fuse_obs_.push_back(omni_obstacles[i].getLocation());
    }

    /** 直接填充足球的信息，不需要融合算法,直接认为球信息完全已知，但是要更新队友球的信息*/
    ball_info_.fuse_ball_ = ball_info_.sensor_ball_[OMNI_BALL];
    teammatesinfo_[AgentID_-1].ball_info_ =ball_info_.fuse_ball_ ;
    ball_info_.ball_info_state_ = SEEBALLBYOWN;
    /** 将其他机器人的足球信息更新，特别是极坐标需要更新*/
    for(int i = 0 ; i < OUR_TEAM; i++)
    {
       nubot::Robot & robot_info = teammatesinfo_[i].robot_info_;
       if(robot_info.isValid() && i+1!=AgentID_)
       {
            BallObject ball_tmp = ball_info_.fuse_ball_;
            DPoint pt=ball_tmp.getGlobalLocation()-robot_info.getLocation();
            PPoint pts(pt);
            ball_tmp.setRealLocation(PPoint(pts.angle_-robot_info.getHead(),pts.radius_));
            teammatesinfo_[i].ball_info_ =ball_tmp;
       }
    }
    publish();
}
/**
    * 发布世界模型信息到机器人上层控制节点（包含队友的信息）；
    * 自身机器人信息、障碍物信息、COACH信息以及队友信息（主要是定位信息、足球信息、策略信息）
    */
void
nubot::World_Model::publish()
{
    /**  将自身感知到的信息以及通信得到的队友信息，发送到上层控制节点*/
    world_model_info_.robotinfo.clear();
    world_model_info_.robotinfo.resize(OUR_TEAM);
    for(std::size_t i = 0 ; i< OUR_TEAM ; i++)
    {
        Robot & robot_info = teammatesinfo_[i].robot_info_;
        world_model_info_.robotinfo[i].AgentID = robot_info.getID();
        world_model_info_.robotinfo[i].pos.x      = robot_info.getLocation().x_;
        world_model_info_.robotinfo[i].pos.y      = robot_info.getLocation().y_;
        world_model_info_.robotinfo[i].heading.theta = robot_info.getHead().radian_;
        world_model_info_.robotinfo[i].vtrans.x   = robot_info.getVelocity().x_;
        world_model_info_.robotinfo[i].vtrans.y   = robot_info.getVelocity().y_;
        world_model_info_.robotinfo[i].isstuck    = robot_info.isStuck();
        world_model_info_.robotinfo[i].iskick     = robot_info.isKickoff();
        world_model_info_.robotinfo[i].isvalid    = robot_info.isValid();
        world_model_info_.robotinfo[i].vrot       = robot_info.getW();
        world_model_info_.robotinfo[i].current_role = robot_info.getCurrentRole();
        world_model_info_.robotinfo[i].role_time =  robot_info.getRolePreserveTime();
        world_model_info_.robotinfo[i].target.x = robot_info.getTarget().x_;
        world_model_info_.robotinfo[i].target.y = robot_info.getTarget().y_;
        world_model_info_.robotinfo[i].isdribble = robot_info.getDribbleState();
    }
    /** 单机器人障碍物，主要用于避障*/
    std::vector< DPoint > tracker;
    obstacles_.getSelfObsTracker(tracker);
    world_model_info_.obstacleinfo.pos.clear();
    world_model_info_.obstacleinfo.pos.resize(tracker.size());
    for(std::size_t i = 0; i< tracker.size() ; i++)
    {
        nubot_common::Point2d point;
        point.x=tracker[i].x_;
        point.y=tracker[i].y_;
        world_model_info_.obstacleinfo.pos[i]= point;
    }
    /** 多机器人障碍物融合结果，主要用于防守跑位，盯人防守等*/
    std::vector< DPoint > opptracker;
    obstacles_.getFuseObsTracker(opptracker);
    world_model_info_.oppinfo.pos.clear();
    world_model_info_.oppinfo.pos.resize(opptracker.size());
    for(std::size_t i = 0; i< opptracker.size() ; i++)
    {
        nubot_common::Point2d point;
        point.x=opptracker[i].x_;
        point.y=opptracker[i].y_;
        world_model_info_.oppinfo.pos[i]= point;
    }

    /** 发布球的信息，自身发布融合之后的足球*/
    world_model_info_.ballinfo.clear();
    world_model_info_.ballinfo.resize(OUR_TEAM);
    for(std::size_t i = 0 ; i< OUR_TEAM ; i++)
    {
        BallObject ball_info;
        if(i == AgentID_-1)
            ball_info=ball_info_.fuse_ball_;
        else
            ball_info = teammatesinfo_[i].ball_info_;
        world_model_info_.ballinfo[i].pos.x =  ball_info.getGlobalLocation().x_;
        world_model_info_.ballinfo[i].pos.y =  ball_info.getGlobalLocation().y_;
        world_model_info_.ballinfo[i].real_pos.angle = ball_info.getRealLocation().angle_.radian_;
        world_model_info_.ballinfo[i].real_pos.radius = ball_info.getRealLocation().radius_;
        world_model_info_.ballinfo[i].velocity.x = ball_info.getVelocity().x_;
        world_model_info_.ballinfo[i].velocity.y = ball_info.getVelocity().y_;
        world_model_info_.ballinfo[i].velocity_known = ball_info.isVelocityKnown();
        world_model_info_.ballinfo[i].pos_known      = ball_info.isLocationKnown();
        if(i == AgentID_-1)
            world_model_info_.ballinfo[i].ballinfostate  = ball_info_.ball_info_state_;
    }

    /** 发布coach的信息*/
    world_model_info_.coachinfo.MatchMode =coach2robot_.MatchMode;
    world_model_info_.coachinfo.MatchType =coach2robot_.MatchType;
    world_model_info_.coachinfo.Passer_pos.x=coach2robot_.Passer_pos.x_;
    world_model_info_.coachinfo.Passer_pos.y=coach2robot_.Passer_pos.y_;
    world_model_info_.coachinfo.Receiver1_pos.x=coach2robot_.Receiver1_pos.x_;
    world_model_info_.coachinfo.Receiver1_pos.y=coach2robot_.Receiver1_pos.y_;
    world_model_info_.coachinfo.Receiver2_pos.x=coach2robot_.Receiver2_pos.x_;
    world_model_info_.coachinfo.Receiver2_pos.y=coach2robot_.Receiver2_pos.y_;
    world_model_info_.coachinfo.isCooperation_ready=coach2robot_.isCooperation_ready;

    worldmodelinfo_pub_.publish(world_model_info_);
}
/**
    * 周期更新队友以及COACH信息（30ms，ROS定时器，COACH等待移植到UBUNTU）
    */
void
nubot::World_Model::updateInfo()            // if the simulation flag is set, this function will not be run
{
    /** 更新所有队友的信息，判断这些信息是否有效，根据时间戳*/
    double min_distance_ball = 100000000;
    teammateIDforBallSelected = -1;
    DPoint ball_vec = DPoint(0,0);
    bool   ball_vec_known = false;
    for( int i = 0 ; i < OUR_TEAM ; i++ )
    {
        if( AgentID_ != i+1 )
        {
            int ltime = DB_get(i+1, TEAMMATESINFO, &teammatesinfo_[i]);
            //接收到的队友信息，并记录其间隔时间，可能表示信息无效；
            teammatesinfo_[i].robot_info_.setlifetime(ltime);
            teammatesinfo_[i].ball_info_.setlifetime(ltime);
            teammatesinfo_[i].robot_info_.update();

            /** 选择机器人于足球最近的作为队友看到的足球，最终会与自身感知到的足球融合*/
            /** 通信没有中断（ltime）并且在获得足球信息的机器人上topic没有延迟太长时间（isValid()）*/
            if(ltime > 0  && ltime < NOT_DATAUPDATE     &&
                    teammatesinfo_[i].ball_info_.isValid()  &&
                    teammatesinfo_[i].robot_info_.isValid())
                teammatesinfo_[i].ball_info_.setValid(true);
            else
                teammatesinfo_[i].ball_info_.setValid(false);

            if( teammatesinfo_[i].ball_info_.isValid() &&  teammatesinfo_[i].ball_info_.isLocationKnown())
            {
                /** 机器人与足球之间的距离，选出其中距离最短的作为当前机器人于足球的距离*/
                if(!ball_vec_known && teammatesinfo_[i].ball_info_.isVelocityKnown()) //
                {
                    ball_vec_known = true;
                    ball_vec = teammatesinfo_[i].ball_info_.getVelocity();
                }
                if(min_distance_ball >  teammatesinfo_[i].ball_info_.getRealLocation().radius_)
                {
                    min_distance_ball = teammatesinfo_[i].ball_info_.getRealLocation().radius_;
                    teammateIDforBallSelected = i;
                    if(teammatesinfo_[i].ball_info_.isVelocityKnown())
                    {
                        ball_vec_known = true;
                        ball_vec = teammatesinfo_[i].ball_info_.getVelocity();
                    }
                }
            }
            //!判断读取的传接球信息是否有效
            if(ltime > 0  && ltime < NOT_DATAUPDATE  &&
                    teammatesinfo_[i].pass_cmds_.isvalid &&
                    teammatesinfo_[i].robot_info_.isValid())
                teammatesinfo_[i].pass_cmds_.isvalid = true;
            else
                teammatesinfo_[i].pass_cmds_.isvalid = false;

            /** 写入障碍物的信息, 必须通信已经建立,将障碍物写入到*/
            std::vector<ObstacleObject> obs_info;
            if(ltime > 0  && ltime < NOT_DATAUPDATE  &&
                    teammatesinfo_[i].robot_info_.isValid())
            {
                for(int j = 0 ; j <MAX_OBSNUMBER_CONST;j++)
                {
                    if(teammatesinfo_[i].obs_info_[j].getPolarLocation().radius_ != 10000)
                        obs_info.push_back(teammatesinfo_[i].obs_info_[j]);
                }
                obstacles_.setOmniObstacles(obs_info,i+1);
            }
            else
                obstacles_.clearOmniObstacles(i+1);
        }
    }

    if(teammateIDforBallSelected!=-1)
    {
        teammatesinfo_[teammateIDforBallSelected].ball_info_.setVelocityKnown(ball_vec_known);
        teammatesinfo_[teammateIDforBallSelected].ball_info_.setVelocity(ball_vec);
    }
    /** 根据接收到的topic更新自身的信息,转换为ms*/

    ros::Time nowtime =  ros::Time::now();
    ros::Duration duration = nowtime - omni_update_time_;
    int self_time = duration.toNSec()/1000000.0;
    teammatesinfo_[AgentID_-1].robot_info_.setlifetime(self_time); //time -> ms
    if(self_time < 0 || self_time > NOT_DATAUPDATE)
        obstacles_.clearOmniObstacles(AgentID_);

    /** 更新全向视觉节点接收topic时间到当前的时间间隔，足球信息*/
    ball_info_.sensor_ball_[OMNI_BALL].setlifetime(self_time);

    /**  下面可能加上障碍物，用于障碍物融合*/

    teammatesinfo_[AgentID_-1].robot_info_.update();
}

bool World_Model::IsLocationInField(DPoint location)
{
    bool rtvl = false;
    static bool isinfiled = false;

    if(!isinfiled && abs(location.x_) < FIELD_LENGTH/2 && abs(location.y_) < FIELD_WIDTH/2)
    {
        isinfiled = true;
    }
    else if(isinfiled && abs(location.x_) < FIELD_LENGTH/2 + LOCATIONERROR && abs(location.y_) < FIELD_WIDTH/2+LOCATIONERROR)
    {
        isinfiled = true;
    }
    else
    {
        isinfiled = false;
    }

    rtvl = isinfiled;
    return rtvl;
}
