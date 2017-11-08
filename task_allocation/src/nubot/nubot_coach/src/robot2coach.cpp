#include <robot2coach.h>

using namespace nubot;

Robot2coach::Robot2coach(char *argv[])
{
    ros::Time::init();
    boost::shared_ptr<ros::NodeHandle> nh_;
    std::string robot_name = argv[1];
    nh_= boost::make_shared<ros::NodeHandle>(robot_name);

    robot2coachinfo_sub_     = nh_->subscribe("/"+robot_name+"2/worldmodel/worldmodelinfo", 1, &Robot2coach::update_info,this);
    passer_allocation_sub_   = nh_->subscribe("/"+robot_name+"2/allocation_info",1,&Robot2coach::update_passerinfo,this);
    receiver1_allocation_sub_= nh_->subscribe("/"+robot_name+"3/allocation_info",1,&Robot2coach::update_receiver1info,this);
    receiver2_allocation_sub_= nh_->subscribe("/"+robot_name+"4/allocation_info",1,&Robot2coach::update_receiver2info,this);

    coach2robotinfo_pub_     = nh_->advertise<nubot_common::CoachInfo>("receive_from_coach",30);
    coach2obs1pos_pub_       = nh_->advertise<nubot_common::Pos2d>("/rival2/obs_position",30);
    coach2obs2pos_pub_       = nh_->advertise<nubot_common::Pos2d>("/rival3/obs_position",30);
    coachinfo_publish_timer_ = nh_->createTimer(ros::Duration(0.03),&Robot2coach::publish,this);
}
Robot2coach::~Robot2coach()
{
}

//用于接受worldmodel的信息
void Robot2coach::update_info(const nubot_common::WorldModelInfo & _world_msg)
{
    //更新机器人信息
    for(int i=0 ; i < OUR_TEAM ; i++)
    {
        robot2coach_info.RobotInfo_[i].setID(_world_msg.robotinfo[i].AgentID);
        robot2coach_info.RobotInfo_[i].setLocation(DPoint(_world_msg.robotinfo[i].pos.x,_world_msg.robotinfo[i].pos.y));
        robot2coach_info.RobotInfo_[i].setHead(Angle(_world_msg.robotinfo[i].heading.theta));
        robot2coach_info.RobotInfo_[i].setVelocity(DPoint( _world_msg.robotinfo[i].vtrans.x,_world_msg.robotinfo[i].vtrans.y));
        robot2coach_info.RobotInfo_[i].setStuck(_world_msg.robotinfo[i].isstuck);
        robot2coach_info.RobotInfo_[i].setKick(_world_msg.robotinfo[i].iskick);
        robot2coach_info.RobotInfo_[i].setValid(_world_msg.robotinfo[i].isvalid);
        robot2coach_info.RobotInfo_[i].setW(_world_msg.robotinfo[i].vrot);
        robot2coach_info.RobotInfo_[i].setDribbleState(_world_msg.robotinfo[i].isdribble);
        robot2coach_info.RobotInfo_[i].setRolePreserveTime(_world_msg.robotinfo[i].role_time);
        robot2coach_info.RobotInfo_[i].setCurrentRole(_world_msg.robotinfo[i].current_role);
        robot2coach_info.RobotInfo_[i].setTarget(DPoint(_world_msg.robotinfo[i].target.x,_world_msg.robotinfo[i].target.y));
    }
    //更新单个机器人障碍物信息
    if(robot2coach_info.Obstacles_[1].size() != 0 && _world_msg.obstacleinfo.pos.size()!=0)
        for(int j=0;j<MAX_OBSNUMBER_CONST;j++)
            robot2coach_info.Obstacles_[1][j]=DPoint( _world_msg.obstacleinfo.pos[j].x, _world_msg.obstacleinfo.pos[j].y);

    //更新融合后障碍物信息
    robot2coach_info.Opponents_.clear();
    for(nubot_common::Point2d point : _world_msg.oppinfo.pos)
        robot2coach_info.Opponents_.push_back(DPoint(point.x,point.y));

    //更新足球物信息
    for(int i = 0 ; i < OUR_TEAM ; i++)
    {
        robot2coach_info.BallInfo_[i].setGlobalLocation(DPoint(_world_msg.ballinfo[i].pos.x ,_world_msg.ballinfo[i].pos.y));
        robot2coach_info.BallInfo_[i].setRealLocation(PPoint(Angle(_world_msg.ballinfo[i].real_pos.angle),_world_msg.ballinfo[i].real_pos.radius));
        robot2coach_info.BallInfo_[i].setVelocity(DPoint(_world_msg.ballinfo[i].velocity.x,_world_msg.ballinfo[i].velocity.y));
        robot2coach_info.BallInfo_[i].setVelocityKnown(_world_msg.ballinfo[i].velocity_known);
        robot2coach_info.BallInfo_[i].setLocationKnown(_world_msg.ballinfo[i].pos_known);
        robot2coach_info.BallInfo_[i].setValid(_world_msg.ballinfo[i].pos_known);
    }
}

void Robot2coach::update_passerinfo(const nubot_common::Allocation_info &_passer_msg)
{
    allocation_info.isPasserLocation[0]=_passer_msg.isPasserLocation;
    allocation_info.isReceiverLocation[0]=_passer_msg.isReceiverLocation;
    allocation_info.passWhich[0]=_passer_msg.passWhich;
    allocation_info.isCooperation_done[0]=_passer_msg.isCooperation_done;
    allocation_info.isNotcatch_ball[0]=_passer_msg.isNotcatch_ball;
}

void Robot2coach::update_receiver1info(const nubot_common::Allocation_info &_receiver_msg)
{
    allocation_info.isPasserLocation[1]=_receiver_msg.isPasserLocation;
    allocation_info.isReceiverLocation[1]=_receiver_msg.isReceiverLocation;
    allocation_info.receiveORnot[1]=_receiver_msg.receiveORnot;
    allocation_info.isCooperation_done[1]=_receiver_msg.isCooperation_done;
    allocation_info.isReceiver_receive[1]=_receiver_msg.isReceiver_receive;
}

void Robot2coach::update_receiver2info(const nubot_common::Allocation_info &_receiver_msg)
{
    allocation_info.isPasserLocation[2]=_receiver_msg.isPasserLocation;
    allocation_info.isReceiverLocation[2]=_receiver_msg.isReceiverLocation;
    allocation_info.receiveORnot[2]=_receiver_msg.receiveORnot;
    allocation_info.isCooperation_done[2]=_receiver_msg.isCooperation_done;
    allocation_info.isReceiver_receive[2]=_receiver_msg.isReceiver_receive;
}

//用于发布coach的信息
void Robot2coach::publish(const ros::TimerEvent &)
{
    coachinfo_publish_info_.MatchMode=coach2robot_info.MatchMode;
    coachinfo_publish_info_.MatchType=coach2robot_info.MatchType;
    coachinfo_publish_info_.isCooperation_ready=coach2robot_info.isCooperation_ready;

    coachinfo_publish_info_.Passer_pos.x=coach2robot_info.Passer_pos.x_;
    coachinfo_publish_info_.Passer_pos.y=coach2robot_info.Passer_pos.y_;
    coachinfo_publish_info_.Receiver1_pos.x=coach2robot_info.Receiver1_pos.x_;
    coachinfo_publish_info_.Receiver1_pos.y=coach2robot_info.Receiver1_pos.y_;
    coachinfo_publish_info_.Receiver2_pos.x=coach2robot_info.Receiver2_pos.x_;
    coachinfo_publish_info_.Receiver2_pos.y=coach2robot_info.Receiver2_pos.y_;

    coach2robotinfo_pub_.publish(coachinfo_publish_info_);

    //the position of obstaclse have changed
    if(obs_positions.obstacle1.x_!=obs_lastpos.x_||obs_positions.obstacle1.y_!=obs_lastpos.y_)
    {
        nubot_common::Pos2d obs_pos;
        obs_lastpos=obs_positions.obstacle1;

        obs_pos.pos.x=obs_positions.obstacle1.x_;
        obs_pos.pos.y=obs_positions.obstacle1.y_;
        coach2obs1pos_pub_.publish(obs_pos);
        obs_pos.pos.x=obs_positions.obstacle2.x_;
        obs_pos.pos.y=obs_positions.obstacle2.y_;
        coach2obs2pos_pub_.publish(obs_pos);
    }
}
void
Robot2coach::run()
{
    ros::spin();
}
