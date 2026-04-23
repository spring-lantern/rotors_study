#include <iostream>
#include <ros/ros.h>
#include <vector>
#include "fake_mod/dynamics.h"

using namespace std;
using namespace modquad;

ros::Subscriber cmd_sub;
ros::Publisher  odom_pub;
ros::Publisher  imu_pub;
ros::Publisher  mesh_pub;
ros::Publisher  state_pub;
ros::Publisher  esc_pub;
ros::Publisher  esc_pub_sim;
ros::ServiceServer  mode_server;
ros::Timer simulate_timer;
ros::Time get_cmdtime;

mavros_msgs::AttitudeTarget immediate_cmd;
vector<mavros_msgs::AttitudeTarget> cmd_buff;
vector<mav_msgs::Actuators> cmd_buff_sim;

XModQuad mod_quad;
bool rcv_cmd = false;

void rcvCmdCallBack(const mavros_msgs::AttitudeTargetConstPtr cmd)
{	
	if (rcv_cmd==false)
	{
		rcv_cmd = true;
		cmd_buff.emplace_back(*cmd);
		get_cmdtime = ros::Time::now();
	}
	else
	{
		cmd_buff.emplace_back(*cmd);
		if ((ros::Time::now() - get_cmdtime).toSec() > mod_quad.getDelay())
		{
			immediate_cmd = cmd_buff[0];
			cmd_buff.erase(cmd_buff.begin());
		}
	}
}

bool rcvSetModeCallBack(mavros_msgs::SetMode::Request& req, mavros_msgs::SetMode::Response& res)
{	
	mod_quad.setState(req.custom_mode);
	res.mode_sent = true;
	return true;
}

void simCallback(const ros::TimerEvent &e)
{
	mod_quad.simOneStep(immediate_cmd);
	odom_pub.publish(mod_quad.getOdom());
	imu_pub.publish(mod_quad.getImu());
	esc_pub.publish(mod_quad.getESC());
	mesh_pub.publish(mod_quad.getMesh());
	state_pub.publish(mod_quad.getState());
}

int main (int argc, char** argv) 
{        
	ros::init(argc, argv, "fake_mod");
	ros::NodeHandle nh("~");

	mod_quad.init(nh);
		
	cmd_sub  = nh.subscribe("cmd", 1000, rcvCmdCallBack);
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
	imu_pub  = nh.advertise<sensor_msgs::Imu>("imu", 10);
	mesh_pub = nh.advertise<visualization_msgs::Marker>("mesh", 10);
	state_pub = nh.advertise<mavros_msgs::State>("state", 10);
	esc_pub = nh.advertise<mavros_msgs::ESCTelemetry>("rpm", 10);
	mode_server = nh.advertiseService("set_mode", rcvSetModeCallBack);

	immediate_cmd.body_rate.x = 0.0;
	immediate_cmd.body_rate.y = 0.0;

	simulate_timer = nh.createTimer(ros::Duration(mod_quad.getTimeResolution()), simCallback);


	ros::spin();

  return 0;
}