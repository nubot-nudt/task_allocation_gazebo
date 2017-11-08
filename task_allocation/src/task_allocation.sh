#!/bin/bash			
source devel/setup.bash
### Get parameters and init
declare -i j
declare -i kill_num
cyan_prefix=$(rosparam get /cyan/prefix)
cyan_num=$(rosparam get /cyan/num)
kill_num=0                                  

### spawn cyan robots
for ((i=1; i<=cyan_num; ++i))
do
    j=$i+1                                              # skip the goal keeper

    # export AGENT=$j
    rosrun world_model       world_model_node ${cyan_prefix}${j}  __name:=${cyan_prefix}_world_model${j} &
    PIDS[kill_num]=$!
    #let "kill_num=kill_num+1"
    
    rosrun nubot_allocation  nubot_allocation ${cyan_prefix}${j}  __name:=${cyan_prefix}_allocation${j} &
    PIDS[kill_num]=$!
    #let "kill_num=kill_num+1"
    sleep 0.5
done 

### run coach_allocation
rosrun  nubot_coach nubot_coach_node ${cyan_prefix} __name:=${cyan_prefix}_coach &
let "kill_num=kill_num+1"

### kill thoes background processes
trap 'kill ${PIDS[*]}' SIGINT
wait

