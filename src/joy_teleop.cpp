#include <mutex>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

namespace IQR
{
class QJoyTeleop
{
public:
  QJoyTeleop();
  ~QJoyTeleop();

private:
  void JoyCallBack(const sensor_msgs::Joy::ConstPtr &joy_msg);
  void CmdPublish();

private:
  std::mutex pubMutex_;

  ros::NodeHandle ph_, nh_;

  ros::Timer pubTimer_;
  ros::Publisher cmdPub_;
  ros::Subscriber joySub_;

  int32_t linearAxisNum_, angularAxisNum_, safeButtonNum_;
  double maxLinearVel_, maxAngularVel_;

  geometry_msgs::Twist lastCmdMsg_;
  bool safeFlage_;
};

QJoyTeleop::QJoyTeleop() : ph_("~"),
                           safeFlage_(false)
{
  ph_.param("linear_axis_num", linearAxisNum_, 1);
  ph_.param("angular_axis_num", angularAxisNum_, 0);
  ph_.param("safe_button_num", safeButtonNum_, 4);
  ph_.param("max_linear_vel", maxLinearVel_, 0.8);
  ph_.param("max_angular_vel", maxAngularVel_, 0.5);

  cmdPub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  joySub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &QJoyTeleop::JoyCallBack, this);

  pubTimer_ = nh_.createTimer(ros::Duration(0.1), std::bind(&QJoyTeleop::CmdPublish, this));
}

QJoyTeleop::~QJoyTeleop()
{
}

void QJoyTeleop::JoyCallBack(const sensor_msgs::Joy::ConstPtr &joy_msg)
{
  geometry_msgs::Twist vel;
  vel.linear.x = maxLinearVel_ * joy_msg->axes[linearAxisNum_];
  vel.angular.z = maxAngularVel_ * joy_msg->axes[angularAxisNum_];
  lastCmdMsg_ = vel;
  if (joy_msg->buttons[safeButtonNum_])
  {
    safeFlage_ = true;
  }
  else
  {
    safeFlage_ = false;
  }
}

void QJoyTeleop::CmdPublish()
{
  static bool pubStopFlage = false;

  std::lock_guard<std::mutex> lck(pubMutex_);

  if (safeFlage_)
  {
    cmdPub_.publish(lastCmdMsg_);
    pubStopFlage = false;
  }
  else if (!safeFlage_ && !pubStopFlage)
  {
    cmdPub_.publish(*new geometry_msgs::Twist());
    pubStopFlage = true;
  }
}

} // namespace IQR


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_teleop");
  IQR::QJoyTeleop joy_teleop;

  ros::spin();
}