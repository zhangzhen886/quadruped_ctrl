### "walking_simulation.py"节点使用说明

#### 订阅的话题：

**"~/cmd_vel"** 包含linear和angular两个子项，其中在运动步态下linear.x和linear.y分别用于控制机器人的前向/侧向移动，angular.x控制旋转的角速度；在站立状态下，linear.x用于控制机器人身体的高度（在标准高度上下浮动），angular.x和angular.y分别控制机器人身体的俯仰（pitch）和横滚（roll）角度。

#### 发布的话题：

**"~/robot_odom"** 机器人里程计，包含全局位姿信息（6维），来自仿真器反馈的真实值。

**"~/imu0"** 6轴姿态信息，包含身体3轴加速度和3轴角速度数据，以及绝对姿态数据（四元素）。

**"~/joint_states"** 机器人12个关节（4*3）的状态信息，包含位置、速度以及扭矩数据（共36维度）。

#### 开启的服务（server）：

**"~/robot_mode"** 选择四足机器人的步态模式，发送req为0表示高性能模式（可选择多种步态类型），1表示低性能模式（自适应行走步态，在standing、walking和troting之间自动切换）。

**"~/gait_type"** 高性能模式下可选择四足机器人的步态类型，其中1为bounding步态，2为pronking步态，4为standing步态，5是trotRunning，7是galloping，8是pacing，9是trotting，10是walking，11是walking2。

**"~/robot_reset"** 发送True重置机器人状态。

#### 调用的服务：

**"~/quad_rl_controller"** req包含4个内容，分别为'pos_data'（3维全局位置+3维姿态欧拉角）、'imu_data'（3维加速度+3维角速度+4维姿态四元素）、'leg_data'（12维关节位置+12维关节速度）和'tau_data'（由MPC计算的12维关节力矩）；result为外部RL算法计算的12维关节力矩，在RL控制模式下将替代MPC对仿真器中的四足机器人进行控制。

