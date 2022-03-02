# Gnc_ardupilot
The following tutorial will show you how to navigate ardupilot using set of waypoints and dynamically avoid the obstacles using 2D-Lidar

## Demo Video at https://youtu.be/L6Iughv_Os8

![snapshot](https://user-images.githubusercontent.com/73148677/156347592-22785172-d9fe-454a-82e2-2cbc5b487204.png)


## Pre-requisites
> Before you clone the repo make sure you install ardupilot and mavros. I'll attach the required links to get you started.

[Adding Ardupilot to your workspace](https://ardupilot.org/dev/docs/building-setup-linux.html)

[Adding Mavros to your workspace](https://docs.px4.io/master/en/ros/mavros_installation.html)

### Clonning GNC repo

>To the the repo into your workspace follwo these steps

```
cd catkin_ws/src
git clone https://github.com/saivinay313/gnc_ardupilot.git
```
> Now Build your workspace
```
cd ~/catkin_ws
catkin build
```
After Successful building your workspace, you have to launch Gazebo world
>Run this command in new terminal
```
roslaunch gnc droneOnly.launch
```
It will take sometime to laod the model(Drone,Lidar etc.) and then you can see the world with drone in it

### Launching SITL and MAVROS 
After launching the world you have to run the SITL 
> Run in new terminal inside folder of the repo
```
./startsitl.sh
```
Wait for the setup get finished and then launch your apm node

>In new terminal
```
roslaunch gnc apm_node.launch
```

This Process will get you read for ruuning your cpp node and to navigate using waypoints

>Run in new terminal 
```
rosrun gnc gnc
```
This node will check if there is a FCU connection and if its connected it will ask user to manually set the drone in guided mode
> to .startsitl.sh termainal and run
```
mode GUIDED
```
### Thats it now drone is now set to navigate through waypoints
>Note: I gave obstacle avoidance threshold to 1metre you can increase and decrease in the gnc.cpp code parameter is **d0**












