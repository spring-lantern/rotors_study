/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include "nav_msgs/Odometry.h"
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
// #include <trajectory_generation_helper/heading_trajectory_helper.h>
// #include <trajectory_generation_helper/polynomial_trajectory_helper.h>
#include "quadrotor_msgs/Trajectory.h"
#include "quadrotor_msgs/TrajectoryPoint.h"
#include "nav_msgs/Path.h"
#include <geometry_msgs/PoseStamped.h>

ros::Subscriber odom_sub, traj_start_trigger_sub;
ros::Publisher trajectory_pub, mpc_trajectory_pub, odom_visualization_pub;
nav_msgs::Odometry odom;
nav_msgs::Path             pathROS;
geometry_msgs::PoseStamped poseROS;

//start pamameters
double desired_height = 0.5;
double start_land_velocity = 0.5;
double start_land_acceleration = 1.0;
double start_idle_duration = 2.0;

Eigen::Vector3d reference_pos_, reference_vel_, reference_acc_;

Eigen::Vector3d desired_position(0.0, 0.0, 1.0);
double take_off_velocity = 0.2;
static bool hover_flag = false;
static bool first_time_start = true;
static bool first_time_hover = true;

bool received_traj_trig = false;
void goToPos(Eigen::Vector3d desired_position, ros::NodeHandle nh){
  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  double desired_yaw = 0.0;
  trajectory_msg.header.stamp = ros::Time::now();
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);
  ros::Duration(0.1).sleep();
  // ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
  //          nh.getNamespace().c_str(),
  //          desired_position.x(),
  //          desired_position.y(),
  //          desired_position.z());
  trajectory_pub.publish(trajectory_msg);
}

void goToMPCPos(Eigen::Vector3d desired_position, Eigen::Vector3d desired_velocity, Eigen::Vector3d desired_acceleration
                , ros::NodeHandle nh){
  quadrotor_msgs::Trajectory mpc_ref_traj;
  quadrotor_msgs::TrajectoryPoint mpc_ref_point;
  double desired_yaw = 0.0;
  mpc_ref_traj.type = 4; //snap
  mpc_ref_traj.header.stamp = ros::Time::now();
  mpc_ref_traj.header.frame_id = "world";

  mpc_ref_point.time_from_start = ros::Duration(0);
  mpc_ref_point.pose.position.x =  desired_position[0];
  mpc_ref_point.pose.position.y =  desired_position[1];
  mpc_ref_point.pose.position.z =  desired_position[2];
  mpc_ref_point.velocity.linear.x =  desired_velocity[0];
  mpc_ref_point.velocity.linear.y =  desired_velocity[1];
  mpc_ref_point.velocity.linear.z =  desired_velocity[2];
  mpc_ref_point.acceleration.linear.x =  desired_acceleration[0];
  mpc_ref_point.acceleration.linear.y =  desired_acceleration[1];
  mpc_ref_point.acceleration.linear.z =  desired_acceleration[2];
  mpc_ref_point.jerk.linear.x =  0;
  mpc_ref_point.jerk.linear.y =  0;
  mpc_ref_point.jerk.linear.z =  0;
  mpc_ref_point.snap.linear.x =  0;
  mpc_ref_point.snap.linear.y =  0;
  mpc_ref_point.snap.linear.z =  0;

  mpc_ref_point.heading = desired_yaw;
  mpc_ref_point.heading_rate = 0;
  mpc_ref_point.heading_acceleration = 0;

  mpc_ref_traj.points.push_back(mpc_ref_point);

  // ros::Duration(0.1).sleep();
  // ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
  //          nh.getNamespace().c_str(),
  //          desired_position.x(),
  //          desired_position.y(),
  //          desired_position.z());
  mpc_trajectory_pub.publish(mpc_ref_traj);
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  received_traj_trig = true;
}

void odometryCallback(const nav_msgs::Odometry& msg){
  odom = msg;
  Eigen::Vector3d odom_pos_visualization(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
  Eigen::Quaterniond odom_q_visualization(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, 
                      odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);

  poseROS.header = msg.header;
  poseROS.header.stamp = msg.header.stamp;
  poseROS.header.frame_id = "world";
  poseROS.pose.position.x = odom_pos_visualization(0);
  poseROS.pose.position.y = odom_pos_visualization(1);
  poseROS.pose.position.z = odom_pos_visualization(2);

  poseROS.pose.orientation.w = odom.pose.pose.orientation.w;
  poseROS.pose.orientation.x = odom.pose.pose.orientation.x;
  poseROS.pose.orientation.y = odom.pose.pose.orientation.y;
  poseROS.pose.orientation.z = odom.pose.pose.orientation.z;
  
  static ros::Time prevt = msg.header.stamp;
  if ((msg.header.stamp - prevt).toSec() > 0.1)
  {
    prevt = msg.header.stamp;
    pathROS.header = poseROS.header;
    pathROS.poses.push_back(poseROS);
    odom_visualization_pub.publish(pathROS);
  }

}

void process(ros::NodeHandle nh){
  static ros::Time start_time = ros::Time::now();

  reference_vel_ << 0.0, 0.0, 0.0;
  reference_acc_ << 0.0, 0.0, 0.0;

  if(odom.pose.pose.position.z >= desired_height){
    hover_flag = true;
  }

  if(first_time_start){
    reference_pos_[0] = odom.pose.pose.position.x;
    reference_pos_[1] = odom.pose.pose.position.y;    
    first_time_start = false;
  }
  if(hover_flag){
    // ROS_INFO("hover!");
    // std::cout<< "hover!" << std::endl;
    if(first_time_hover){
      reference_pos_[0] = odom.pose.pose.position.x;
      reference_pos_[1] = odom.pose.pose.position.y;
      reference_pos_[2] = odom.pose.pose.position.z;
      first_time_hover = false;
    }
    // reference_pos_[2] = desired_height;
    goToMPCPos(reference_pos_, reference_vel_, reference_acc_, nh);
    return;
  }

  // std::cout<< "start!" << std::endl;



  reference_pos_[2] = start_land_velocity * (ros::Time::now() - start_time).toSec();
  reference_vel_[2] = start_land_velocity;
  if ((ros::Time::now() - start_time).toSec()  < start_land_velocity / start_land_acceleration) {
    reference_acc_[2] = start_land_acceleration;
    reference_vel_[2] = start_land_acceleration * (ros::Time::now() - start_time).toSec();
  } else {
    reference_acc_[2] = 0;
  }

  // reference_vel[0] = 0;
  // reference_vel[1] = 0;
  // reference_vel[2] = 0;
  // if ((ros::Time::now() - start_time).toSec() < start_land_velocity / start_land_acceleration) {
  //   reference_acc[2] = start_land_acceleration;
  //   reference_vel[2] = start_land_acceleration *
  //       (ros::Time::now() - start_time).toSec();
  // } 
  goToMPCPos(reference_pos_, reference_vel_, reference_acc_, nh);


}

int main(int argc, char** argv){
  ros::init(argc, argv, "hovering_example");
  ros::NodeHandle nh;
  
  trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);



  mpc_trajectory_pub =
      nh.advertise<quadrotor_msgs::Trajectory>(
      "/TABV/command/mpc_trajectory", 10);
  odom_visualization_pub = 
      nh.advertise<nav_msgs::Path>("odom_visualization_path", 10);
  // odom_sub = nh.subscribe("/TABV/vi_sensor/ground_truth/odometry", 1, odometryCallback);
  // odom_sub = nh.subscribe("/TABV/odometry_sensor1/odometry", 1, odometryCallback);
  odom_sub = nh.subscribe("/TABV/ground_truth/odometry", 1, odometryCallback, ros::TransportHints().tcpNoDelay());
  traj_start_trigger_sub = nh.subscribe("/move_base_simple/goal", 1, goalCallback, ros::TransportHints().tcpNoDelay());
  ROS_INFO("Started hovering example.");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  }
  else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();

  /* init vins*/
  // Eigen::Vector3d desired_position;

  
  Eigen::Vector3d desired_position(0.0, 0.0, 1.0);
  if(!received_traj_trig){
    //takeoff and hover
    goToPos(desired_position, nh);
  }


  // goToMPCPos(desired_position, nh);

  // reference_pos_ << 0.0, 0.0, 0.0;
  // reference_vel_ << 0.0, 0.0, 0.0;
  // reference_acc_ << 0.0, 0.0, 0.0;

  //start to desired_position
  // ros::Rate r(100.0);
  // while (ros::ok())
  // {
  //     r.sleep();
  //     ros::spinOnce();
  //     process(nh);
  //     // ROS_ERROR("!!!!");
  // }

  // if(abs(odom.pose.pose.position.z - 1.0) > 1e-2 && !hover_flag){
  //   goToMPCPos(desired_position, nh);
  //    std::cout << "hovering!!!!!!!!!!!!!!!!!!" << std::endl;
  // }
  // else{
  //   hover_flag = true;
  // }


  // up
  // for (int i=0; i<num; i++){
  //   desired_position = Eigen::Vector3d(0.0, 0.0, num*step);
  //   goToMPCPos(desired_position, nh);
  //   // goToPos(desired_position, nh);
  // }
  // // left
  // for (int i=0; i<num; i++){
  //   desired_position = Eigen::Vector3d(0.0, num*step, 1.0);
  //   goToMPCPos(desired_position, nh);
  // }
  // //right
  // for (int i=0; i<num; i++){
  //   desired_position = Eigen::Vector3d(0.0, 1.0 - 2*num*step, 1.0);
  //   goToMPCPos(desired_position, nh);
  // }
  // // back to center
  // for (int i=0; i<num; i++){
  //   desired_position = Eigen::Vector3d(0.0, num*step - 1.0, 1.0);
  //   goToMPCPos(desired_position, nh);
  // }
  // // forward
  // for (int i=0; i<num; i++){
  //   desired_position = Eigen::Vector3d(num*step, 0.0, 1.0);
  //   goToMPCPos(desired_position, nh);
  // }
  // // back to center
  // for (int i=0; i<num; i++){
  //   desired_position = Eigen::Vector3d(1.0 - num*step, 0.0, 1.0);
  //   goToMPCPos(desired_position, nh);
  // }

  ros::spin();
  return 0;
}
