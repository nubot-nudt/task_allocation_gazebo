#include "nubot/nubot_allocation/task_allocation.hpp"

Taskallocation::Taskallocation(int argc, char **argv)
{
    const char * environment;
    std::string robot_name;
    std::string str = argv[1];
    std::string str2 = str.substr(str.size()-1);
    environment = str2.c_str();
    robot_name = str;
    ROS_INFO("nubot_control: robot_name:%s",robot_name.c_str());
    nh_ = boost::make_shared<ros::NodeHandle>(robot_name);

    ballhandle_client_  = nh_->serviceClient<nubot_common::BallHandle>("BallHandle");
    shoot_client_       = nh_->serviceClient<nubot_common::Shoot>("Shoot");
    motor_cmd_pub_      = nh_->advertise<nubot_common::Teleop_joy>("nubotcontrol/teleop_joy",1);
    allocation_info_pub_= nh_->advertise<nubot_common::Allocation_info>("allocation_info",1);
    worldmodelinfo_sub_ = nh_->subscribe("worldmodel/worldmodelinfo", 1, &Taskallocation::update_world_model_info,this);
    control_timer_      = nh_->createTimer(ros::Duration(0.03),&Taskallocation::loopControl,this);

    m_behaviour_=new Behaviour(worldmodel_info_.Obstacles_);
    worldmodel_info_.AgentID_ = atoi(environment);
    isReceiver_location=false;
    isPasser_location=false;
    isCooperation_Ready=false;
    isCooperation_done =false;
    isReceiver_receive=false;
    isNotcatch_ball=false;
    isfirst_random=true;
    isfirst_static=true;
    isfirst_dynamic=true;

    pass_which=false;
    receive_not=false;

    float _costFun_1[2][8]={{9,5,3,1,4,2,4,3},
                            {5,3,4,2,3,3,7,5}};
    float _costFun_2[2][8]={{3,3,7,5,5,3,4,2},
                            {4,2,4,3,9,5,3,1}};
    for(int i=0;i<2;i++)
        for(int j=0;j<8;j++)
        {
            costFun_1[i][j]=_costFun_1[i][j];
            costFun_2[i][j]=_costFun_2[i][j];
        }
}

Taskallocation::~Taskallocation()
{
    m_behaviour_->app_vx_ = 0;
    m_behaviour_->app_vy_ = 0;
    m_behaviour_->app_w_  = 0;
    setEthercatCommond();
}

void Taskallocation::update_world_model_info(const nubot_common::WorldModelInfo &_world_msg)
{
    /** 更新PathPlan自身与队友的信息，自身的策略信息记住最好不要更新，因为本身策略是从此传过去的*/
    for(std::size_t i = 0 ; i < OUR_TEAM ; i++)
    {
        worldmodel_info_.RobotInfo_[i].setID(_world_msg.robotinfo[i].AgentID);
        worldmodel_info_.RobotInfo_[i].setLocation(DPoint(_world_msg.robotinfo[i].pos.x,_world_msg.robotinfo[i].pos.y));
        worldmodel_info_.RobotInfo_[i].setHead(Angle(_world_msg.robotinfo[i].heading.theta));
        worldmodel_info_.RobotInfo_[i].setVelocity(DPoint(_world_msg.robotinfo[i].vtrans.x,_world_msg.robotinfo[i].vtrans.y));
        worldmodel_info_.RobotInfo_[i].setStuck(_world_msg.robotinfo[i].isstuck);
        worldmodel_info_.RobotInfo_[i].setKick(_world_msg.robotinfo[i].iskick);
        worldmodel_info_.RobotInfo_[i].setValid(_world_msg.robotinfo[i].isvalid);
        worldmodel_info_.RobotInfo_[i].setW(_world_msg.robotinfo[i].vrot);
        /** 信息是来源于队友，则要更新机器人策略信息*/
        if(worldmodel_info_.AgentID_ != i+1)
        {
            worldmodel_info_.RobotInfo_[i].setDribbleState(_world_msg.robotinfo[i].isdribble);
            worldmodel_info_.RobotInfo_[i].setRolePreserveTime(_world_msg.robotinfo[i].role_time);
            worldmodel_info_.RobotInfo_[i].setCurrentRole(_world_msg.robotinfo[i].current_role);
            worldmodel_info_.RobotInfo_[i].setTarget(DPoint(_world_msg.robotinfo[i].target.x,_world_msg.robotinfo[i].target.y));
        }
    }
    /** 更新障碍物信息*/
    worldmodel_info_.Obstacles_.clear();
    for(nubot_common::Point2d point : _world_msg.obstacleinfo.pos )
        worldmodel_info_.Obstacles_.push_back(DPoint(point.x,point.y));
    worldmodel_info_.Opponents_.clear();
    for(nubot_common::Point2d point : _world_msg.oppinfo.pos )
        worldmodel_info_.Opponents_.push_back(DPoint(point.x,point.y));
    /** 更新足球物信息*/
    for(std::size_t i = 0 ; i < OUR_TEAM ; i++)
    {
        worldmodel_info_.BallInfo_[i].setGlobalLocation(DPoint(_world_msg.ballinfo[i].pos.x ,_world_msg.ballinfo[i].pos.y));
        worldmodel_info_.BallInfo_[i].setRealLocation(PPoint(Angle(_world_msg.ballinfo[i].real_pos.angle),_world_msg.ballinfo[i].real_pos.radius));
        worldmodel_info_.BallInfo_[i].setVelocity(DPoint(_world_msg.ballinfo[i].velocity.x,_world_msg.ballinfo[i].velocity.y));
        worldmodel_info_.BallInfo_[i].setVelocityKnown(_world_msg.ballinfo[i].velocity_known);
        worldmodel_info_.BallInfo_[i].setLocationKnown(_world_msg.ballinfo[i].pos_known);
        worldmodel_info_.BallInfo_[i].setValid(_world_msg.ballinfo[i].pos_known);
    }
    worldmodel_info_.BallInfoState_ = _world_msg.ballinfo[worldmodel_info_.AgentID_-1].ballinfostate;
    /** 更新的COACH信息*/
    worldmodel_info_.CoachInfo_.MatchMode = _world_msg.coachinfo.MatchMode;
    worldmodel_info_.CoachInfo_.MatchType = _world_msg.coachinfo.MatchType;
    worldmodel_info_.CoachInfo_.Passer_pos.x_  = _world_msg.coachinfo.Passer_pos.x;
    worldmodel_info_.CoachInfo_.Passer_pos.y_  = _world_msg.coachinfo.Passer_pos.y;
    worldmodel_info_.CoachInfo_.Receiver1_pos.x_  = _world_msg.coachinfo.Receiver1_pos.x;
    worldmodel_info_.CoachInfo_.Receiver1_pos.y_  = _world_msg.coachinfo.Receiver1_pos.y;
    worldmodel_info_.CoachInfo_.Receiver2_pos.x_  = _world_msg.coachinfo.Receiver2_pos.x;
    worldmodel_info_.CoachInfo_.Receiver2_pos.y_  = _world_msg.coachinfo.Receiver2_pos.y;
    worldmodel_info_.CoachInfo_.isCooperation_ready=_world_msg.coachinfo.isCooperation_ready;

    isCooperation_Ready=worldmodel_info_.CoachInfo_.isCooperation_ready;
    Passer_pos=worldmodel_info_.CoachInfo_.Passer_pos;
    Receiver1_pos=worldmodel_info_.CoachInfo_.Receiver1_pos;
    Receiver2_pos=worldmodel_info_.CoachInfo_.Receiver2_pos;


}

void Taskallocation::loopControl(const ros::TimerEvent &event)
{
    static ros::Time start_time;
    static char last_MatchMode = worldmodel_info_.CoachInfo_.MatchMode;

    if(worldmodel_info_.CoachInfo_.MatchMode == STARTROBOT)
    {
        if(worldmodel_info_.AgentID_==2)
        {
            if(worldmodel_info_.RobotInfo_[worldmodel_info_.AgentID_-1].getDribbleState())
                start_time= ros::Time::now();
            ros::Duration notDribbleDuration = ros::Time::now() - start_time;               //the duration that the Passer do not dribble ball
            if(notDribbleDuration.sec>4)
            {
                isPasser_location=false;
                isNotcatch_ball=true;
                isCooperation_done=true;
            }

            processPasser();
        }
        else if(worldmodel_info_.AgentID_==3||worldmodel_info_.AgentID_==4)
        {
            if(!worldmodel_info_.RobotInfo_[2].getDribbleState()&&!worldmodel_info_.RobotInfo_[3].getDribbleState())
                start_time= ros::Time::now();
            else
                isReceiver_receive=true;

            ros::Duration DribbleDuration = ros::Time::now() - start_time;               //the duration that the Receivers dribble ball
            if(DribbleDuration.sec>2)
            {
                isReceiver_location=false;
                isCooperation_done=true;
                if(worldmodel_info_.RobotInfo_[worldmodel_info_.AgentID_-1].getDribbleState())
                {
                    shoot.request.strength=worldmodel_info_.RobotInfo_[worldmodel_info_.AgentID_-1].getLocation().distance(Passer_pos)/300;;
                    shoot.request.ShootPos=1;
                    ballhandle.request.enable=0;
                }
            }

            processReceiver();
        }

        if(isCooperation_done)
        {
            isfirst_random=true;
            isfirst_static=true;
            isfirst_dynamic=true;
        }
    }
    else
    {
        stopRobot();
        ballhandle.request.enable = 0;
        shoot.request.strength=0;
        isfirst_random=true;
        isfirst_static=true;
        isfirst_dynamic=true;
        start_time = ros::Time::now();
    }

    last_MatchMode = worldmodel_info_.CoachInfo_.MatchMode;
    ballhandle_client_.call(ballhandle);
    if(shoot.request.strength>0)
    {
        shoot_client_.call(shoot);
        shoot.request.strength=0;
    }
    worldmodel_info_.checkDribble(ballhandle.response.BallIsHolding);
    setEthercatCommond();
    pubAllocation_info();
}

void Taskallocation::setEthercatCommond()
{
    nubot_common::Teleop_joy  command;
    command.Vx = m_behaviour_->app_vx_ ;
    command.Vy = m_behaviour_->app_vy_ ;
    command.w  = m_behaviour_->app_w_  ;

    m_behaviour_->app_vx_=0;
    m_behaviour_->app_vy_=0;
    m_behaviour_->app_w_=0;

    motor_cmd_pub_.publish(command);
}

void Taskallocation::pubAllocation_info()
{
    nubot_common::Allocation_info info;
    info.passWhich=pass_which;
    info.receiveORnot=receive_not;
    info.isPasserLocation=isPasser_location;
    info.isReceiverLocation=isReceiver_location;
    info.isCooperation_done=isCooperation_done;
    info.isReceiver_receive=isReceiver_receive;
    info.isNotcatch_ball=isNotcatch_ball;

    allocation_info_pub_.publish(info);
}

void Taskallocation::stopRobot()
{
    m_behaviour_->app_vx_ = m_behaviour_->app_vy_ =  0.0;
    m_behaviour_->app_w_  =  0;
}

void Taskallocation::processPasser()
{
    DPoint robot_pos  = worldmodel_info_.RobotInfo_[worldmodel_info_.AgentID_-1].getLocation();
    Angle  robot_ori  = worldmodel_info_.RobotInfo_[worldmodel_info_.AgentID_-1].getHead();
    DPoint ball_pos   = worldmodel_info_.BallInfo_[worldmodel_info_.AgentID_-1].getGlobalLocation();
    double thetaofr2b = ball_pos.angle(robot_pos).radian_;
    double thetaofr2p = Passer_pos.angle(robot_pos).radian_;
    //Passer does not dribble, catch ball
    if(isCooperation_Ready)
    {
        isCooperation_done=false;        
        if(worldmodel_info_.CoachInfo_.MatchType==Random_Choice&&isfirst_random)
            pass_which=randomChoice();
        else if(isfirst_static)
        {
            calculateP();
            pass_which=passWhich();
        }

        if(pass_which)                      //pass forward
        {
            double thetaofp2r=worldmodel_info_.RobotInfo_[receive_assistID-1].getLocation().angle(Passer_pos).radian_;
            if(fabs(thetaofp2r-robot_ori.radian_)>0.1)
                m_behaviour_->rotate2AbsOrienation(2,4,thetaofp2r,MAXW,robot_ori);
            else if(worldmodel_info_.RobotInfo_[worldmodel_info_.AgentID_-1].getDribbleState())
            {
                shoot.request.strength=worldmodel_info_.RobotInfo_[receive_assistID-1].getLocation().distance(Passer_pos)/200;
                shoot.request.ShootPos=1;
                ballhandle.request.enable=0;
            }
        }
        else                                //pass back
        {
            double thetaofp2r=worldmodel_info_.RobotInfo_[receive_defenseID-1].getLocation().angle(Passer_pos).radian_;
            if(fabs(thetaofp2r-robot_ori.radian_)>0.1)
                m_behaviour_->rotate2AbsOrienation(2,4,thetaofp2r,MAXW,robot_ori);
            else if(worldmodel_info_.RobotInfo_[worldmodel_info_.AgentID_-1].getDribbleState())
            {
                shoot.request.strength=worldmodel_info_.RobotInfo_[receive_defenseID-1].getLocation().distance(Passer_pos)/200;
                shoot.request.ShootPos=1;
                ballhandle.request.enable=0;
            }
        }
    }
    else if(!worldmodel_info_.RobotInfo_[worldmodel_info_.AgentID_-1].getDribbleState())
    {
        m_behaviour_->move2Positionwithobs(1,2,ball_pos,MAXVEL,robot_pos,robot_ori,false);
        m_behaviour_->rotate2AbsOrienation(2,4,thetaofr2b,MAXW,robot_ori);
        ballhandle.request.enable = 1;
        isPasser_location=false;
    }
    else if(robot_pos.distance(Passer_pos)>LOCATIONERROR)
    {
        m_behaviour_->rotate2AbsOrienation(2,4,thetaofr2p,MAXW,robot_ori);
        if(fabs(thetaofr2p-robot_ori.radian_)<1)
            m_behaviour_->move2Positionwithobs(1,2,Passer_pos,MAXVEL,robot_pos,robot_ori,true);
        ballhandle.request.enable = 1;
        isPasser_location=false;
    }
    else
        isPasser_location=true;
}

void Taskallocation::processReceiver()
{
    DPoint2s robot_pos  = worldmodel_info_.RobotInfo_[worldmodel_info_.AgentID_-1].getLocation();
    Angle    robot_ori  = worldmodel_info_.RobotInfo_[worldmodel_info_.AgentID_-1].getHead();
    DPoint2s ball_pos   = worldmodel_info_.BallInfo_[worldmodel_info_.AgentID_-1].getGlobalLocation();
    DPoint2s ball_vel   = worldmodel_info_.BallInfo_[worldmodel_info_.AgentID_-1].getVelocity();
    double   thetaofr2b   = ball_pos.angle(robot_pos).radian_;
    double   ballveltheta = ball_vel.angle().radian_;
    double   thetaofb2r   = robot_pos.angle(ball_pos).radian_;
    double   theta_d      = ballveltheta - thetaofb2r;
    theta_d = angularnorm(theta_d);
    DPoint2s receiver_pos;

    if(worldmodel_info_.AgentID_==3)
        receiver_pos=Receiver1_pos;
    else if(worldmodel_info_.AgentID_==4)
        receiver_pos=Receiver2_pos;

    if(isCooperation_Ready)
    {
        isCooperation_done=false;
        if(worldmodel_info_.CoachInfo_.MatchType==Random_Choice&&isfirst_random)
            receive_not=randomChoice();
        else if(worldmodel_info_.CoachInfo_.MatchType==Static_Game&&isfirst_static || worldmodel_info_.CoachInfo_.MatchType==Dynamic_Game&&isfirst_dynamic)
        {
            calculateP();
            receive_not=receiveORnot();
        }

        if(receive_not)                 //receive
        {
            double thetaofr2p=Passer_pos.angle(receiver_pos).radian_;
            DPoint p  = pglobal2rel(ball_pos,ballveltheta,robot_pos);
            DPoint p2 = prel2global(ball_pos,ballveltheta,DPoint(p.x_,0.0));

            m_behaviour_->rotate2AbsOrienation(2,4,thetaofr2p,MAXW,robot_ori);

            if(!worldmodel_info_.RobotInfo_[passID-1].getDribbleState()&&fabs(theta_d)<deg(20)&&!worldmodel_info_.RobotInfo_[worldmodel_info_.AgentID_-1].getDribbleState())
            {
                isReceiver_receive=false;
                m_behaviour_->move2Positionwithobs(2,4,p2,MAXVEL,robot_pos,robot_ori,false);
                m_behaviour_->rotate2AbsOrienation(2,4,thetaofr2p,MAXW,robot_ori);
                ballhandle.request.enable = 1;
            }
        }
        else
            isReceiver_receive=false;
    }

    else if(robot_pos.distance(receiver_pos)>LOCATIONERROR)
    {
        m_behaviour_->move2Positionwithobs(1,2,receiver_pos,MAXVEL,robot_pos,robot_ori,true);
        m_behaviour_->rotate2AbsOrienation(2,4,thetaofr2b,MAXW,robot_ori);
        isReceiver_location=false;
        ballhandle.request.enable = 0;
    }
    else
        isReceiver_location=true;
}

void Taskallocation::calculateP()
{
    passID=2;
    if(Receiver1_pos.x_>0)
    {
        receive_assistID=3;
        receive_defenseID=4;
    }
    else
    {
        receive_assistID=4;
        receive_defenseID=3;
    }

    DPoint2s ball_pos     = worldmodel_info_.BallInfo_[worldmodel_info_.AgentID_-1].getGlobalLocation();
    DPoint2s ball_vel     = worldmodel_info_.BallInfo_[worldmodel_info_.AgentID_-1].getVelocity();
    DPoint2s pass_pos     = worldmodel_info_.RobotInfo_[passID-1].getLocation();
    DPoint2s receive_pos_A= worldmodel_info_.RobotInfo_[receive_assistID-1].getLocation();
    DPoint2s receive_pos_D= worldmodel_info_.RobotInfo_[receive_defenseID-1].getLocation();

    std::vector <DPoint2s> obstacle_pos;
    for(int i=0;i<worldmodel_info_.Opponents_.size();i++)
        obstacle_pos.push_back(worldmodel_info_.Opponents_[i]);
    if(obstacle_pos.size()<2)
    {
        ROS_ERROR("there are not enough obstacles");
        return;
    }
    float dis_P2R_A=pass_pos.distance(receive_pos_A);
    float dis_P2R_D=pass_pos.distance(receive_pos_D);
    float dis_R2O_A=min(receive_pos_A.distance(obstacle_pos[0]),receive_pos_A.distance(obstacle_pos[1]));
    float dis_R2O_D=min(receive_pos_D.distance(obstacle_pos[0]),receive_pos_D.distance(obstacle_pos[1]));

    float P_Aa=dis_R2O_A/dis_P2R_A;
    float P_Ac=dis_P2R_A/dis_R2O_A;
    float P_Da=dis_P2R_D/dis_R2O_D;
    float P_Dc=dis_R2O_D/dis_P2R_D;

    if(worldmodel_info_.CoachInfo_.MatchType==Dynamic_Game && worldmodel_info_.AgentID_!=passID && !worldmodel_info_.RobotInfo_[passID-1].getDribbleState()&&ball_vel.norm()>50)
    {
        double   ballveltheta = ball_vel.angle().radian_;
        double   thetaofb2A   = receive_pos_A.angle(ball_pos).radian_;
        double   thetaofb2D   = receive_pos_D.angle(ball_pos).radian_;
        double   theta_A      = ballveltheta - thetaofb2A;
        double   theta_D      = ballveltheta - thetaofb2D;
        theta_A = angularnorm(theta_A);
        theta_D = angularnorm(theta_D);
        P_Aa=P_Aa*(M_PI-fabs(theta_A))/M_PI;
        P_Ac=P_Ac*fabs(theta_A)/M_PI;
        P_Da=P_Da*fabs(theta_D)/M_PI;
        P_Dc=P_Dc*(M_PI-fabs(theta_D))/M_PI;
        isfirst_dynamic=false;
    }
    isfirst_static=false;
    float P_A=P_Aa+P_Ac;
    float P_D=P_Da+P_Dc;

    P_a=P_Aa+P_Da;
    P_c=P_Ac+P_Dc;
    P_a_A=P_Aa/P_A;
    P_c_A=P_Ac/P_A;
    P_a_D=P_Da/P_D;
    P_c_D=P_Dc/P_D;
}

bool Taskallocation::randomChoice()
{
    srand(clock());
    bool choice=rand()%2;
    isfirst_random=false;

    if(choice==1)
        return true;
    else
        return false;
}

bool Taskallocation::passWhich()
{
    if(P_a>P_c)
        return true;                  //pass forward
    else
        return false;                 //pass back
}

bool Taskallocation::receiveORnot()
{
    DPoint2s robot_pos  = worldmodel_info_.RobotInfo_[worldmodel_info_.AgentID_-1].getLocation();
    int max_row0,max_row2;
    int max_col0,max_col2;
    float max_row0_val=0;
    float max_row2_val=0;
    if(robot_pos.x_>0)              //assist
    {
        float result_A[4][4]=
        {costFun_1[0][0]*P_a_A+costFun_1[0][4]*P_c_A,costFun_1[0][1]*P_a_A+costFun_1[0][5]*P_c_A,costFun_1[0][2]*P_a_A+costFun_1[0][6]*P_c_A,costFun_1[0][3]*P_a_A+costFun_1[0][7]*P_c_A,
         costFun_1[0][0]*P_a_A+costFun_1[1][4]*P_c_A,costFun_1[0][1]*P_a_A+costFun_1[1][5]*P_c_A,costFun_1[0][2]*P_a_A+costFun_1[1][6]*P_c_A,costFun_1[0][3]*P_a_A+costFun_1[1][7]*P_c_A,
         costFun_1[1][0]*P_a_A+costFun_1[0][4]*P_c_A,costFun_1[1][1]*P_a_A+costFun_1[0][5]*P_c_A,costFun_1[1][2]*P_a_A+costFun_1[0][6]*P_c_A,costFun_1[1][3]*P_a_A+costFun_1[0][7]*P_c_A,
         costFun_1[1][0]*P_a_A+costFun_1[1][4]*P_c_A,costFun_1[1][1]*P_a_A+costFun_1[1][5]*P_c_A,costFun_1[1][2]*P_a_A+costFun_1[1][6]*P_c_A,costFun_1[1][3]*P_a_A+costFun_1[1][7]*P_c_A};

//        std::cout<<"assist"<<std::endl;
//        std::cout<<result_A[0][0]<<" "<<result_A[0][1]<<" "<<result_A[0][2]<<" "<<result_A[0][3]<<std::endl;
//        std::cout<<result_A[1][0]<<" "<<result_A[1][1]<<" "<<result_A[1][2]<<" "<<result_A[1][3]<<std::endl;
//        std::cout<<result_A[2][0]<<" "<<result_A[2][1]<<" "<<result_A[2][2]<<" "<<result_A[2][3]<<std::endl;
//        std::cout<<result_A[3][0]<<" "<<result_A[3][1]<<" "<<result_A[3][2]<<" "<<result_A[3][3]<<std::endl;

        for(int i=0;i<4;i++)
            if(result_A[i][0]>max_row0_val)
            {
                max_row0_val=result_A[i][0];
                max_row0=i;
            }
        for(int j=0;j<4;j++)
            if(result_A[j][2]>max_row2_val)
            {
                max_row2_val=result_A[j][2];
                max_row2=j;
            }
        result_A[max_row0][1]>result_A[max_row0][3]? max_col0=1 : max_col0=3;
        result_A[max_row2][1]>result_A[max_row2][3]? max_col2=1 : max_col2=3;

        if(max_col0==1&&max_col2==3)
        {
            if(result_A[max_row0][max_col0]>result_A[max_row2][max_col2])
                return true;
            else
                return false;
        }
        else if(max_col0==1)
            return true;
        else if(max_col2==3)
            return false;
        else
            return false;
    }
    else                           //defense
    {
        float result_D[4][4]=
        {costFun_2[0][0]*P_a_D+costFun_2[0][4]*P_c_D,costFun_2[0][1]*P_a_D+costFun_2[0][5]*P_c_D,costFun_2[0][2]*P_a_D+costFun_2[0][6]*P_c_D,costFun_2[0][3]*P_a_D+costFun_2[0][7]*P_c_D,
         costFun_2[0][0]*P_a_D+costFun_2[1][4]*P_c_D,costFun_2[0][1]*P_a_D+costFun_2[1][5]*P_c_D,costFun_2[0][2]*P_a_D+costFun_2[1][6]*P_c_D,costFun_2[0][3]*P_a_D+costFun_2[1][7]*P_c_D,
         costFun_2[1][0]*P_a_D+costFun_2[0][4]*P_c_D,costFun_2[1][1]*P_a_D+costFun_2[0][5]*P_c_D,costFun_2[1][2]*P_a_D+costFun_2[0][6]*P_c_D,costFun_2[1][3]*P_a_D+costFun_2[0][7]*P_c_D,
         costFun_2[1][0]*P_a_D+costFun_2[1][4]*P_c_D,costFun_2[1][1]*P_a_D+costFun_2[1][5]*P_c_D,costFun_2[1][2]*P_a_D+costFun_2[1][6]*P_c_D,costFun_2[1][3]*P_a_D+costFun_2[1][7]*P_c_D};

//        std::cout<<"defense"<<std::endl;
//        std::cout<<result_D[0][0]<<" "<<result_D[0][1]<<" "<<result_D[0][2]<<" "<<result_D[0][3]<<std::endl;
//        std::cout<<result_D[1][0]<<" "<<result_D[1][1]<<" "<<result_D[1][2]<<" "<<result_D[1][3]<<std::endl;
//        std::cout<<result_D[2][0]<<" "<<result_D[2][1]<<" "<<result_D[2][2]<<" "<<result_D[2][3]<<std::endl;
//        std::cout<<result_D[3][0]<<" "<<result_D[3][1]<<" "<<result_D[3][2]<<" "<<result_D[3][3]<<std::endl;

        for(int i=0;i<4;i++)
            if(result_D[i][0]>max_row0_val)
            {
                max_row0_val=result_D[i][0];
                max_row0=i;
            }
        for(int j=0;j<4;j++)
            if(result_D[j][2]>max_row2_val)
            {
                max_row2_val=result_D[j][2];
                max_row2=j;
            }
        result_D[max_row0][1]>result_D[max_row0][3]? max_col0=1 : max_col0=3;
        result_D[max_row2][1]>result_D[max_row2][3]? max_col2=1 : max_col2=3;

        if(max_col0==1&&max_col2==3)
        {
            if(result_D[max_row0][max_col0]>result_D[max_row2][max_col2])
                return true;
            else
                return false;
        }
        else if(max_col0==1)
            return true;
        else if(max_col2==3)
            return false;
        else
            return false;
    }

}
