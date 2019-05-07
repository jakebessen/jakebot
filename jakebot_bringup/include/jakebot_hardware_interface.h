
#pragma once

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/Empty.h>
// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
// ostringstream
#include <sstream>

const unsigned int NUM_JOINTS = 2;

/// \brief Hardware interface for a robot
class JakebotInterface : public hardware_interface::RobotHW
{
public:
  JakebotInterface();

  void write() {
    double diff_ang_speed_left = cmd[0];
    double diff_ang_speed_right = cmd[1];
    limitDifferentialSpeed(diff_ang_speed_left, diff_ang_speed_right);
	// Publish results
	std_msgs::Float32MultiArray wheel_vel_msg;
	wheel_vel_msg.data.push_back(diff_ang_speed_left);
	wheel_vel_msg.data.push_back(diff_ang_speed_right);
    wheel_vel_pub_.publish(wheel_vel_msg);
  }

  /**
   * Reading encoder values and setting position and velocity of enconders
   */
  void read(const ros::Duration &perid) {
    double ang_distance_left = _wheel_angle[0];
    double ang_distance_right = _wheel_angle[1];
    pos[0] += ang_distance_left;
    vel[0] += ang_distance_left;
    pos[1] += ang_distance_right;
    vel[1] += ang_distance_right;
    /*
    std::ostringstream os;
    for (unsigned int i = 0; i < NUM_JOINTS - 1; ++i)
    {
      os << cmd[i] << ", ";
      pos[i] = cmd[i];
    }
    os << cmd[NUM_JOINTS - 1];
    ROS_INFO_STREAM("Commands for joints: " << os.str());
    */
  }

  ros::Time get_time() {
    prev_update_time = curr_update_time;
    curr_update_time = ros::Time::now();
    return curr_update_time;
  }

  ros::Duration get_period() {
    return curr_update_time - prev_update_time;
  }

  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;

  double cmd[NUM_JOINTS];
  double pos[NUM_JOINTS];
  double vel[NUM_JOINTS];
  double eff[NUM_JOINTS];

  bool running_;
  double _wheel_diameter;
  double _max_speed;
  double _wheel_angle[NUM_JOINTS];

  ros::Time curr_update_time, prev_update_time;

  ros::Subscriber wheel_angle_sub_;
  ros::Publisher wheel_vel_pub_;

  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;

  bool start_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = true;
    return true;
  }

  bool stop_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = false;
    return true;
  }

  void WheelAngleCallback(const std_msgs::Float32MultiArray& msg) {
    _wheel_angle[1] = msg.data[1];
    _wheel_angle[0] = msg.data[0];
  }

  void limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right)
  {
	double speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));
	if (speed > _max_speed) {
		diff_speed_left *= _max_speed / speed;
		diff_speed_right *= _max_speed / speed;
	}
  }

};  // class

JakebotInterface::JakebotInterface()
: running_(true)
  , private_nh("~")
  , start_srv_(nh.advertiseService("start", &JakebotInterface::start_callback, this))
  , stop_srv_(nh.advertiseService("stop", &JakebotInterface::stop_callback, this))
  {
    private_nh.param<double>("wheel_diameter", _wheel_diameter, 0.064);
    private_nh.param<double>("max_speed", _max_speed, 1.0);

    // Intialize raw data
    std::fill_n(pos, NUM_JOINTS, 0.0);
    std::fill_n(vel, NUM_JOINTS, 0.0);
    std::fill_n(eff, NUM_JOINTS, 0.0);
    std::fill_n(cmd, NUM_JOINTS, 0.0);

    // connect and register the joint state and velocity interfaces
    for (unsigned int i = 0; i < NUM_JOINTS; ++i)
    {
      std::ostringstream os;
      if( i == 0 ){
        os << "left_wheel_joint";
      }
      else if(i ==1 ){
        os << "right_wheel_joint";
      }

      hardware_interface::JointStateHandle state_handle(os.str(), &pos[i], &vel[i], &eff[i]);
      jnt_state_interface.registerHandle(state_handle);

      hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(os.str()), &cmd[i]);
      jnt_vel_interface.registerHandle(vel_handle);
    }
    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_vel_interface);


	// Initialize publishers and subscribers
	wheel_vel_pub_ = nh.advertise<std_msgs::Float32MultiArray>("wheel_vel", 1);
	wheel_angle_sub_ = nh.subscribe("wheel_angle", 1, &JakebotInterface::WheelAngleCallback, this);

}
