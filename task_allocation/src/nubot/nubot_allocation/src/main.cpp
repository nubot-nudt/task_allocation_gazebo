#include <nubot/nubot_allocation/task_allocation.hpp>
using namespace nubot;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"task_allocation_node");
    ros::Time::init();
    ROS_INFO("start task_allocation proces");
    Taskallocation task_allocation(argc,argv);
    ros::spin();
    return 0;
}

