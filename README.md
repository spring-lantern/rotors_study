# Px4Ctrl_Gazebo

这是一个将Gazebo接入实验室常用的Px4Ctrl的代码。

## 快速开始

* 编译：

  ```
  catkin_make -DCATKIN_WHITELIST_PACKAGES="quadrotor_msgs"
  catkin_make -DCATKIN_WHITELIST_PACKAGES="mav_msgs"
  catkin_make -DCATKIN_WHITELIST_PACKAGES="" -DCMAKE_BUILD_TYPE=Release
  ```
  卡(catkin_make -DCATKIN_WHITELIST_PACKAGES="" -DCMAKE_BUILD_TYPE=Release -j2)

* 运行：

source devel/setup.bash



  ```
  sh src/scripts/px4ctrl_bazi.sh
  ```

   
  先用"Publish Point"点一下坐标系，然后点击"2D Pose Estimate"，飞机悬停。再点"3D Nav Goal", 跟八字。

## 代码简要介绍

* px4ctrl: 和实验室仓库的px4ctrl的区别是，加入了参数`use_sim=true`，在仿真中接收Gazebo发出的Odom和Imu。并添加了角速度环到力矩环的反馈控制（原来这个步骤是交给飞控的）。
* rotors_simulator: 四旋翼Gazebo仿真。仿真模型相关参数主要在`/rotors_simulator/rotors_description/urdf/TABV_multirotor_base.xacro`、`/rotors_simulator/rotors_description/urdf/TABV_base.xacro`和`/rotors_simulator/rotors_description/urdf/TABV.xacro`中修改。四旋翼的Odom为`/TABV/odometry_sensor1/odometry`。
* fake_rc：模拟遥控器节点，接入手柄或rviz指令，转发为`/mavros/rc/in`。如果有遥控器的使用者，可以在 `run_mpc.launch` 中令`use_joy=true`开启遥控器模式，和实物中的操作一样，5通道悬停，6通道飞八字。
* fake_mod：模拟飞机节点，发布odom和其他传感器、飞控等信息，主要原理是用4-5阶的龙格库塔方法积分状态方程。


