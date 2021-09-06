# quadruped_robot

### MIT mini cheetah simulation in pybullet
MIT mini cheetah use customized simulator and lcm framework, which is not a popular way to do the robot development. Now, we extract the algorithm and do the simulation using ros and pybullet. This can be simple to deploy the system into different custom robot or plantform, and easy to learn the algorithm.

<img src="https://github.com/Derek-TH-Wang/quadruped_ctrl/blob/master/quadruped_balance.gif" alt="show" />
<img src="https://github.com/Derek-TH-Wang/quadruped_ctrl/blob/master/vision.png" alt="show" />
<img src="https://github.com/Derek-TH-Wang/quadruped_ctrl/blob/master/rviz.png" alt="show" />

### System requirements:
Ubuntu 18.04, ROS Mellodic  

### Dependency:
use Logitech gamepad to control robot  
```
git clone https://github.com/Derek-TH-Wang/gamepad_ctrl.git
```

### Build
```
cd {your workspace}
catkin make
source devel/setup.bash
```

### Terrain
you can modify the ```config/quadruped_ctrl_cinfig.yaml/terrain``` to deploy different terrains, there are four terrains supported in the simulator now, for example:
```
"plane"
"stairs"
"random1"
"random2"
"racetrack"
```

### Running:
run the gamepad node to control robot:
```
roslaunch gamepad_ctrl gamepad_ctrl.launch
```
run the controller in simulator:  
```
roslaunch quadruped_ctrl quadruped_ctrl.launch
```

switch the camera on / off:
camera set ```True``` or ```False``` in ```config/quadruped_ctrl_config.yaml```, then launch the rviz to see the point cloud:
```
roslaunch quadruped_ctrl vision.launch
```

also can switch the gait type:  
```
rosservice call /gait_type "cmd: 1"
```

gait type:
```
0:trot
1:bunding
2:pronking
3:random
4:standing
5:trotRunning
6:random2
7:galloping
8:pacing
9:trot (same as 0)
10:walking
11:walking2
```

### Motion Data

启动名为"quad_rl_controller"的ros server，能够在回调函数中获取到机器人的运动状态数据，数据内容包含以下两个数组：

"imu_date": length 10，

"leg_data": length 24，[0:12]为关节**位置**（弧度制），[12:24]为关节的**角速度**。分别对应**右前腿**关节"fr_abad"、"fr_thigh"、"fr_knee"，**左前腿**关节"fl_abad"、"fl_thigh"、"fl_knee"，**右后腿**关节，以及**左后腿**关节。