#include "fake_mod/dynamics.h"

namespace modquad
{
    void XModQuad::simOneStep(mavros_msgs::AttitudeTarget cmd)
    {
        if (fcu_state.mode == "GUIDED_NOGPS")
        {
            // get rpm
            rpm[0] = floor(cmd.body_rate.x);
            rpm[1] = (cmd.body_rate.x - rpm[0]) * 1000.0;
            rpm[2] = floor(cmd.body_rate.y);
            rpm[3] = (cmd.body_rate.y - rpm[2]) * 1000.0;
            for (size_t i=0; i<4; i++)
            {
                rpm[i] = params.p1 * rpm[i] * rpm[i] + params.p2 * rpm[i] + params.p3;
            }
            rpm += guassRandom4d(params.noise_rpm);
            for (size_t i=0; i<4; i++)
            {
                rpm[i] = std::min(std::max(rpm[i], params.min_rpm), params.max_rpm);
            }

            // integrate by Runge–Kutta
            double dt_2 = params.time_resolution / 2.0;
            State k1 = getDiff(now_state);
            State k2 = getDiff(now_state + k1 * dt_2);
            State k3 = getDiff(now_state + k2 * dt_2);
            State k4 = getDiff(now_state + k3 * params.time_resolution);
            now_state = now_state + (k1 + k2*2 + k3*2 + k4) * (params.time_resolution/6.0);

            imu.linear_acceleration.x = k1.v.x();
            imu.linear_acceleration.y = k1.v.y();
            imu.linear_acceleration.z = k1.v.z() + params.g;
        }
        
        // change odom, imu and marker
        Eigen::Vector3d now_pos = now_state.p + guassRandom3d(params.noise_pos);
        Eigen::Vector3d now_vel = now_state.v + guassRandom3d(params.noise_vel);
        Eigen::Vector3d now_omega = now_state.omega;
        Eigen::Vector3d now_acc = Eigen::Vector3d(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
        Eigen::Quaterniond now_q(now_state.R);
        now_q.normalize();
        now_acc = now_state.R.transpose() * now_acc;

        odom.header.stamp = ros::Time::now();
        odom.pose.pose.position.x = now_pos(0);
        odom.pose.pose.position.y = now_pos(1);
        odom.pose.pose.position.z = now_pos(2);
        odom.pose.pose.orientation.w = now_q.w();
        odom.pose.pose.orientation.x = now_q.x();
        odom.pose.pose.orientation.y = now_q.y();
        odom.pose.pose.orientation.z = now_q.z();
        odom.twist.twist.linear.x = now_vel(0);
        odom.twist.twist.linear.y = now_vel(1);
        odom.twist.twist.linear.z = now_vel(2);
        odom.twist.twist.angular.x = now_omega(0);
        odom.twist.twist.angular.y = now_omega(1);
        odom.twist.twist.angular.z = now_omega(2);
 
        imu.header.stamp = ros::Time::now();
        imu.orientation = odom.pose.pose.orientation;
        imu.angular_velocity.x = now_omega(0);
        imu.angular_velocity.y = now_omega(1);
        imu.angular_velocity.z = now_omega(2);
        imu.linear_acceleration.x = now_acc(0);
        imu.linear_acceleration.y = now_acc(1);
        imu.linear_acceleration.z = now_acc(2);
        
        marker.header.stamp = ros::Time::now();
        marker.pose = odom.pose.pose;

        esc_tele.header.stamp = ros::Time::now();
        for (size_t i=0; i<4; i++)
        {
            double rpm_temp = now_state.rotor_angular_rate[i] * 30.0 / M_PI;
            esc_tele.esc_telemetry[i].rpm = int32_t(rpm_temp);
        }
    }

    State XModQuad::getDiff(const State &state) const
    {
        State state_dot;

        // Re-orthonormalize R (polar decomposition)
        Eigen::LLT<Eigen::Matrix3d> llt(state.R.transpose() * state.R);
        Eigen::Matrix3d             P = llt.matrixL();
        Eigen::Matrix3d             R = state.R * P.inverse();

        // rotor drag variables
        Eigen::Vector3d             D(params.rotor_drag_dx, params.rotor_drag_dy, params.rotor_drag_dz);
        Eigen::Vector3d             rotor_drag = -R*D.asDiagonal()*R.transpose()*state.v;
        double                      v_h = state.v.dot(R.col(0)+R.col(1));
        double                      drag_f = params.rotor_drag_kh * v_h * v_h;

        // get thrust and moment
        Eigen::Vector4d now_rpm = state.rotor_angular_rate * 30.0 / M_PI;
        Eigen::Vector4d thrusts = params.kf * now_rpm.array().square();
        Eigen::Vector4d thrust_moment = params.mix_matrix * thrusts;

        // add angular acceleration of rotors and gyroscopic effects
        Eigen::Vector3d             moment = thrust_moment.tail(3);
        Eigen::Vector4d             tau_x_gyro(-state.omega(1), -state.omega(1), state.omega(1), state.omega(1));
        Eigen::Vector4d             tau_y_gyro(state.omega(0), state.omega(0), -state.omega(0), -state.omega(0));
        Eigen::Vector4d             tau_z_rotor_acc(-1.0, -1.0, 1.0, 1.0);
        Eigen::Vector4d             rotor_angular_acc = (rpm * M_PI / 30.0 - state.rotor_angular_rate) / params.rotor_time_constant;
        
        moment(0) += params.Ip * tau_x_gyro.dot(now_rpm);
        moment(1) += params.Ip * tau_y_gyro.dot(now_rpm);
        moment(2) += params.Ip * tau_z_rotor_acc.dot(rotor_angular_acc * 30.0 / M_PI);
        moment    += Eigen::Vector3d::Constant(params.ext_tau);

        Eigen::Matrix3d omega_vee(Eigen::Matrix3d::Zero());
        omega_vee(2, 1) = state.omega(0);
        omega_vee(1, 2) = -state.omega(0);
        omega_vee(0, 2) = state.omega(1);
        omega_vee(2, 0) = -state.omega(1);
        omega_vee(1, 0) = state.omega(2);
        omega_vee(0, 1) = -state.omega(2);
        
        state_dot.p = state.v;
        state_dot.v = -Eigen::Vector3d(0, 0, params.g) + (thrust_moment(0) + drag_f) * R.col(2) / params.mass + rotor_drag;
        state_dot.R = R * omega_vee;
        state_dot.omega = params.J.inverse() * (moment - state.omega.cross(params.J * state.omega));
        state_dot.rotor_angular_rate = rotor_angular_acc;
        
        return state_dot;
    }
}
