#pragma once

#include <iostream>
#include <random>
#include <math.h>
#include <string>

#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ESCTelemetry.h>
#include <mavros_msgs/ESCTelemetryItem.h>
#include <mav_msgs/Actuators.h>

namespace modquad
{
  struct Param 
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    double          g;
    double          mass;
    double          kf;
    double          zmoment_thrust_coeff;
    double          p1;
    double          p2;
    double          p3;
    double          arm_length;
    double          Ip;
    double          max_rpm;
    double          min_rpm;
    double          init_x;
    double          init_y;
    double          init_yaw;
    double          ctrl_delay;
    double          rotor_time_constant;
    double          time_resolution;
    Eigen::Matrix3d J;
    Eigen::Matrix4d mix_matrix;
    double          rotor_drag_dx;
    double          rotor_drag_dy;
    double          rotor_drag_dz;
    double          rotor_drag_kh;
    double          noise_pos;
    double          noise_vel;
    double          noise_rpm;
    double          ext_tau;
  };

  struct State 
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();
    Eigen::Vector4d rotor_angular_rate = Eigen::Vector4d::Zero();

    inline State operator+(const State &t) const 
    {
      State sum;
      sum.p = p + t.p;
      sum.v = v + t.v;
      sum.R = R + t.R;
      sum.omega = omega + t.omega;
      sum.rotor_angular_rate = rotor_angular_rate + t.rotor_angular_rate;

      return sum;
    }

    inline State operator*(const double &t) const 
    {
      State mul;
      mul.p = p * t;
      mul.v = v * t;
      mul.R = R * t;
      mul.omega = omega * t;
      mul.rotor_angular_rate = rotor_angular_rate * t;

      return mul;
    }
  };

  // Inputs are Dshot for the motors
  // Rotor numbering is:
  // 3   1 
  //   ↑
  // 2   4
  // with 1 and 2 counter-clockwise and 3 and 4 clockwise (looking from top)
  class XModQuad 
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    private:
      Param params;
      State now_state;
      sensor_msgs::Imu imu;
      nav_msgs::Odometry odom;
      visualization_msgs::Marker marker;
      mavros_msgs::State fcu_state;
      mavros_msgs::ESCTelemetry esc_tele;
      std::normal_distribution<double> distribution{0.0,1.0};
      std::default_random_engine generator;
      Eigen::Vector4d rpm;

    public:
      XModQuad() {};

      ~XModQuad() {};

      inline void init(ros::NodeHandle& nh)
      {
        // init params
        std::vector<double> iner_; 
        nh.getParam("simulator/gravity", params.g);
        nh.getParam("simulator/mass", params.mass);
        nh.getParam("simulator/kf", params.kf);
        nh.getParam("simulator/zmoment_thrust_coeff", params.zmoment_thrust_coeff);
        nh.getParam("simulator/p1", params.p1);
        nh.getParam("simulator/p2", params.p2);
        nh.getParam("simulator/p3", params.p3);
        nh.getParam("simulator/arm_length", params.arm_length);
        nh.getParam("simulator/Ip", params.Ip);
        nh.getParam("simulator/max_rpm", params.max_rpm);
        nh.getParam("simulator/min_rpm", params.min_rpm);
        nh.getParam("simulator/init_x", params.init_x);
        nh.getParam("simulator/init_y", params.init_y);
        nh.getParam("simulator/init_yaw", params.init_yaw);
        nh.getParam("simulator/ctrl_delay", params.ctrl_delay);
        nh.getParam("simulator/rotor_time_constant", params.rotor_time_constant);
        nh.getParam("simulator/time_resolution", params.time_resolution);
        nh.getParam("simulator/inertia", iner_);
        nh.getParam("simulator/rotor_drag_dx", params.rotor_drag_dx);
        nh.getParam("simulator/rotor_drag_dy", params.rotor_drag_dy);
        nh.getParam("simulator/rotor_drag_dz", params.rotor_drag_dz);
        nh.getParam("simulator/rotor_drag_kh", params.rotor_drag_kh);
        nh.getParam("simulator/noise_pos", params.noise_pos);
        nh.getParam("simulator/noise_vel", params.noise_vel);
        nh.getParam("simulator/noise_rpm", params.noise_rpm);
        nh.getParam("simulator/ext_tau", params.ext_tau);

        for (size_t i=0; i<3; i++)
        {
          for (size_t j=0; j<3; j++)
          {
            params.J(i, j) = iner_[3*i + j];
          }
        }

        double thrust_arm = params.arm_length / sqrt(2);
        Eigen::Matrix4d mix_temp;
        mix_temp << 1.0, 1.0, 1.0, 1.0, \
                    -thrust_arm, thrust_arm, thrust_arm, -thrust_arm,\
                    -thrust_arm, thrust_arm, -thrust_arm, thrust_arm,\
                    -params.zmoment_thrust_coeff, -params.zmoment_thrust_coeff, params.zmoment_thrust_coeff, params.zmoment_thrust_coeff;
        params.mix_matrix = mix_temp;

        now_state.p = Eigen::Vector3d(params.init_x, params.init_y, 0.0);
        now_state.R = Eigen::Matrix3d(Eigen::Quaterniond(cos(params.init_yaw/2.0), 0.0, 0.0, sin(params.init_yaw/2.0)));

        // init msgs
        odom.header.frame_id = "world";
        imu.header.frame_id = "world";
        fcu_state.mode = "STABILIZE";
        fcu_state.connected = true;
        esc_tele.header.frame_id = "world";
        for (size_t i=0; i<4; i++)
        {
          mavros_msgs::ESCTelemetryItem it;
          it.rpm = 0;
          esc_tele.esc_telemetry.push_back(it);
        }
        marker.header.frame_id = "world";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = params.init_x;
        marker.pose.position.y = params.init_y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = cos(params.init_yaw/2.0);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = sin(params.init_yaw/2.0);
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.mesh_resource = "package://fake_mod/meshes/mod_quad.mesh";

        return;
      }

      inline double getDelay()
      {
        return params.ctrl_delay;
      }

      inline double getTimeResolution()
      {
        return params.time_resolution;
      }

      inline sensor_msgs::Imu getImu()
      {
        return imu;
      }

      inline nav_msgs::Odometry getOdom()
      {
        return odom;
      }

      inline visualization_msgs::Marker getMesh()
      {
        return marker;
      }

      inline mavros_msgs::State getState()
      {
        return fcu_state;
      }

      inline mavros_msgs::ESCTelemetry getESC()
      {
        return esc_tele;
      }

      inline void setState(std::string mode)
      {
        fcu_state.mode = mode;
        return;
      }

      inline Eigen::Vector3d guassRandom3d(double std)
      {
        return std * Eigen::Vector3d(distribution(generator), distribution(generator), distribution(generator));
      }

      inline Eigen::Vector4d guassRandom4d(double std)
      {
        return std * Eigen::Vector4d(distribution(generator), distribution(generator), distribution(generator), distribution(generator));
      }

      void simOneStep(mavros_msgs::AttitudeTarget cmd);

      State getDiff(const State &state) const;
  };
}
