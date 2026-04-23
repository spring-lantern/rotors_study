#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/RCIn.h>


/*
static 限制作用域
constexpr 编译期常量(编译的时候就确定了，不能改) 如constexpr int a = 3;那a编译完就变成了3
uint32_t   无符号32位整数(0到2^23-1)

static constexpr uint32_t Mode = 6;
定义一个 编译期常量整数 6，名字叫 Mode，属于 fakerc 命名空间

由于mavros  这里的定义等于物理通道-1
mavros      实际飞机
0通道横滚     1 
1通道俯仰     2
3通道偏航     4

4通道档位     5
6通道模式     7

*/

namespace fakerc 
{
  static constexpr uint32_t Mode = 6; // up = -1.0, mid = 0.0, down = 1.0
  static constexpr uint32_t Gear = 4; // up = -1.0, down = 1.0 
  static constexpr uint32_t W1 = 3;   // yaw_channel, left = 1.0, right = -1.0
  static constexpr uint32_t W2 = 1;   // pitch_channel, up = -1.0, down = 1.0
  static constexpr uint32_t W3 = 0;   // roll_channel, left = 1.0, right = -1.0
}

/*
是一个典型：ROS 节点类封装模式
      类
      ├── 成员变量（订阅/发布/状态）
      ├── 构造函数（初始化 ROS）
      ├── 回调函数（处理输入）
      ├── 主循环（输出控制）

*/


/*
FakeRC类  这个模块负责   把 joystick / rviz 输入 → 转成 RC 信号
就两部分
  私有
  公有
*/


/*
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber fakerc_sub;
    ros::Subscriber ch5_sub;
    ros::Subscriber ch6_sub;
    ros::Publisher fakerc_pub;
    ros::WallTimer loop_timer;
   
ros::NodeHandle 表示：来自 ros 命名空间下的 NodeHandle 类型。
sensor_msgs::Joy 表示：来自 sensor_msgs 命名空间下的 Joy 类型
以上是私有成员变量的声明，在构造函数中被赋值(完成初始化)
如fakerc_sub = nh_.subscribe("joy", 10, &FakeRC::joyCallback, this);   
*/

/*
FakeRC(const ros::NodeHandle& nh, const ros::NodeHandle& pnh):
        nh_(nh), pnh_(pnh),last_msg_time()

const ros::NodeHandle& nh — 以常量引用的方式接收一个 NodeHandle 对象（避免拷贝，禁止修改）
《C++ primer plus 第六版中文版》8章8.2引用变量 p210


单冒号的语法是“成员初始化列表”，
《C++ primer plus 第六版中文版》12章p379
将nh_赋值为nh
pnh_赋值为pnh
last_msg_time后面的括号里什么都没有，会被C++调用默认构造函数初始化为0
*/




class FakeRC 
{
  private:
    ros::NodeHandle nh_, pnh_;
    //在一行中同时声明两个 NodeHandle 对象，变量之间用逗号分隔。
    //下划线后缀命名是 C++ 中用于私有成员变量的通用命名规范，与函数内部的局部变量、函数参数区分开。
    ros::Subscriber fakerc_sub;
    ros::Subscriber ch5_sub;
    ros::Subscriber ch6_sub;
    ros::Publisher fakerc_pub;
    ros::WallTimer loop_timer;
    sensor_msgs::Joy fakerc_cmd;
    ros::Time last_msg_time;

    double joystick_timeout_ = 0.5;
    double zero_tolerance = 0.05;
    bool use_joy = false;
    bool use_sim = false;

  public:

    //构造函数
    FakeRC(const ros::NodeHandle& nh, const ros::NodeHandle& pnh):
        nh_(nh), pnh_(pnh),last_msg_time()
    {
      fakerc_cmd = sensor_msgs::Joy();
      fakerc_cmd.axes = std::vector<float>(8, 0);
      fakerc_cmd.buttons = std::vector<int32_t>(8, 0);

      pnh_.getParam("fake_rc/zero_tolerance", zero_tolerance);
      pnh_.getParam("fake_rc/use_joy", use_joy);
      pnh_.getParam("fake_rc/use_sim", use_sim);
/*
先是launch中给到ROS参数服务器，
 <param name="fake_rc/use_joy" value="true"/>
用pnh_.getParam("fake_rc/use_joy", use_joy);将值传递给后面的use_joy
当然上面的bool use_joy = false;只是一个默认值，只有当getParam无法找到对应参数的时候才会生效

然后在下面的
if (use_joy)
发挥作用
*/


      fakerc_pub = nh_.advertise<mavros_msgs::RCIn>("rcin", 100);

      if (use_joy)
      {
        // 订阅手柄话题，使用主循环
        fakerc_sub = nh_.subscribe("joy", 10, &FakeRC::joyCallback, this);
        loop_timer = nh_.createWallTimer(ros::WallDuration(0.1), &FakeRC::mainLoop, this);
      }
      else
      {
        // 订阅RViz鼠标点击话题，使用RViz循环
        ch5_sub = nh_.subscribe("/initialpose", 10, &FakeRC::ch5CallBack, this);
        ch6_sub = nh_.subscribe("/clicked_point", 10, &FakeRC::ch6CallBack, this);
        loop_timer = nh_.createWallTimer(ros::WallDuration(0.1), &FakeRC::rvizLoop, this);
      }

    }


    //(委托构造函数)构造函数调用另一个构造函数
    FakeRC():FakeRC(ros::NodeHandle(),ros::NodeHandle("~")){ }



    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) 
    {
      fakerc_cmd = *msg;
      last_msg_time = ros::Time::now();
    }



    void ch5CallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
    {
      if (fakerc_cmd.axes[fakerc::Mode] < -0.5)
      {
        fakerc_cmd.axes[fakerc::Mode] = 0.0;
      }
      else if (fakerc_cmd.axes[fakerc::Mode] > -0.5)
      {
        fakerc_cmd.axes[fakerc::Mode] = -1.0;
      }

      return;
    }

    void ch6CallBack(const geometry_msgs::PointStampedConstPtr& msg)
    {
      if (fakerc_cmd.axes[fakerc::Gear] < -0.5)
      {
        fakerc_cmd.axes[fakerc::Gear] = 1.0;
      }
      else if (fakerc_cmd.axes[fakerc::Gear] > -0.5)
      {
        fakerc_cmd.axes[fakerc::Gear] = -1.0;
      }
      return;
    }

    bool fakercAvailable() 
    {
      if ((ros::Time::now() - last_msg_time) > ros::Duration(joystick_timeout_)) 
      {
        return false;
      }

      return true;
    }

    void mainLoop(const ros::WallTimerEvent& time) 
    {
      mavros_msgs::RCIn rc_msg;
      rc_msg.header.frame_id = "world";
      rc_msg.header.stamp = ros::Time::now();
      for (size_t i=0; i<4; i++)
        rc_msg.channels.push_back(1500);
      for (size_t i=4; i<8; i++)
        rc_msg.channels.push_back(1000);

      if (fakercAvailable()) 
      {
        rc_msg.channels[4] = (uint)(-1000.0 * fakerc_cmd.axes[fakerc::Mode] + 1000);
        rc_msg.channels[5] = (uint)(-1000.0 * fakerc_cmd.axes[fakerc::Gear] + 1000);

        if (fabs(fakerc_cmd.axes[fakerc::W1]) > zero_tolerance)
        {
          rc_msg.channels[fakerc::W1] = (uint)(fakerc_cmd.axes[fakerc::W1] * 500.0 + 1500);
        }

        if (fabs(fakerc_cmd.axes[fakerc::W2]) > zero_tolerance) 
        {
          rc_msg.channels[fakerc::W2] = (uint)(fakerc_cmd.axes[fakerc::W2] * 500.0 + 1500);
        }

        if (fabs(fakerc_cmd.axes[fakerc::W3]) > zero_tolerance) 
        {
          rc_msg.channels[fakerc::W3] = (uint)(fakerc_cmd.axes[fakerc::W3] * 500.0 + 1500);
        }
      }

      fakerc_pub.publish(rc_msg);
    }

    void rvizLoop(const ros::WallTimerEvent& time)
    {
      mavros_msgs::RCIn rc_msg;
      rc_msg.header.frame_id = "world";
      rc_msg.header.stamp = ros::Time::now();
      for (size_t i=0; i<4; i++)
        rc_msg.channels.push_back(1500);
      for (size_t i=4; i<8; i++)
        rc_msg.channels.push_back(1000);

      rc_msg.channels[4] = (uint)(-1000.0 * fakerc_cmd.axes[fakerc::Mode] + 1000);
      rc_msg.channels[5] = (uint)(-1000.0 * fakerc_cmd.axes[fakerc::Gear] + 1000);

      if (fabs(fakerc_cmd.axes[fakerc::W1]) > zero_tolerance)
      {
        rc_msg.channels[fakerc::W1] = (uint)(fakerc_cmd.axes[fakerc::W1] * 500.0 + 1500);
      }

      if (fabs(fakerc_cmd.axes[fakerc::W2]) > zero_tolerance) 
      {
        rc_msg.channels[fakerc::W2] = (uint)(fakerc_cmd.axes[fakerc::W2] * 500.0 + 1500);
      }

      if (fabs(fakerc_cmd.axes[fakerc::W3]) > zero_tolerance) 
      {
        rc_msg.channels[fakerc::W3] = (uint)(fakerc_cmd.axes[fakerc::W3] * 500.0 + 1500);
      }

      fakerc_pub.publish(rc_msg);
    }
};



int main(int argc, char** argv) 
{
  ros::init(argc, argv, "fake_rc_node");
  
  FakeRC fake_rc;
  
  ros::spin();
  
  return 0;
}
