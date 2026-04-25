# study_simulation

### author: Zhang Yuqing

First, install a package that converts remote controller data into ROS topics.

>sudo apt update && sudo apt install ros-noetic-joy

After connecting the remote controller to the computer, run the following command to confirm that the controller port is recognized.
>ls /dev/input/js*

Start the following joy_node independently to ensure the data is properly recognized.
>source /opt/ros/noetic/setup.zsh
>roscore
>rosrun joy joy_node _dev:=/dev/input/js0
>rostopic echo /joy


Referencing the simulation setup from experienced developers, the launch command is as follows.

```bash
roslaunch    rotors_gazebo     simulator_test_mpc.launch
roslaunch    px4ctrl           run_ctrl.launch
```


