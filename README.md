# task_allocation_gazebo
The code for task allocation and the simulation system based on ROS and Gazebo for task allocation are included.

## Runtime environment

1. Ubuntu 16.04
2. ROS Kinetic
3. Gazebo version 7.0 (the full ROS Kinetic includes the Gazebo 7.0)

## Build

Place the package in your workspace content.

`$ cd task_allocation_gazebo/task_allocation/`

`$ catkin_make`

`$ cd ../gazebo_visual/`

`$ catkin_make`

## Run

### gazebo_visual

The workspace is for Gazebo visualization, and you can launch the Gazebo first by using the following command. 

`$ source gazebo_visual/devel/setup.sh`

`$ roslaunch nubot_gazebo game_ready.launch`

![](image/Gazebo.png)

**Notice:** the program is modified in the match program, so it is not perfect. If the program starts error, please try again.

### task_allocation

The Qt UI for control terminal and the code for task allocation are included in this workspace, and you can launch the robot's process and control terminal by using following command. Before that you must launch the gazebo_visual first.

`$  ./task_allocation/src/task_allocation.sh `

![](image/Coach.png)

**Instructions for use:**  

1. **Start: ** the robots will carry out the cooperation in the same scene with the same method which you choose
2. **Auto: ** the robots will carry out the cooperation with three methods one by one and change the positions of robots and obstacles randomly after the every methods have to be completed. Then the next set of experiments will be carried out in a new scene.
3. **Result_show: ** the data (1|0,1|0,1|0) represents the action selection and the following 1|0 represents whether the Receiver receives the ball.  

## Close

You can destroy the GUI with the close button on the applications, but the all processes have to be closed by Ctrl+C in the terminal.