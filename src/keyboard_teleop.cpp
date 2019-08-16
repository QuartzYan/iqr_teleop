#include <map>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

namespace IQR
{
std::map<char, std::vector<float>> moveStatus{
    {'w', {1, 0, 0}},
    {'s', {-1, 0, 0}},
    {'a', {0, 0, 1}},
    {'d', {0, 0, -1}},
    {'q', {0, 1, 0}},
    {'e', {0, -1, 0}},
    {' ', {0, 0, 0}}};

class QKeyBoardTeleop
{
public:
  QKeyBoardTeleop();
  ~QKeyBoardTeleop();

private:
  void PubCmdLoop();
  void Stop();
  void PrintInfo();
  int getch();

private:
  ros::NodeHandle ph_, nh_;
  ros::Publisher cmdPub_;
  int32_t pubRate_;
  double maxLinearVel_, maxAngularVel_;
};

QKeyBoardTeleop::QKeyBoardTeleop() : ph_("~")
{
  ph_.param("max_linear_vel", maxLinearVel_, 0.3);
  ph_.param("max_angular_vel", maxAngularVel_, 0.3);

  cmdPub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

  PubCmdLoop();
}

QKeyBoardTeleop::~QKeyBoardTeleop()
{
}

void QKeyBoardTeleop::PrintInfo()
{
  std::cout << "Reading from keyboard" << std::endl;
  std::cout << "Use W-S-A-D-Q-E keys to control the robot" << std::endl;
  std::cout << "Use Space keys to stop the robot" << std::endl;
  std::cout << "Ctrl-C to ecit!" << std::endl;
}

void QKeyBoardTeleop::Stop()
{
  cmdPub_.publish(*new geometry_msgs::Twist());
}

void QKeyBoardTeleop::PubCmdLoop()
{
  geometry_msgs::Twist twist;
  double x(0), y(0), th(0);
  int key(' ');
  PrintInfo();
  while (ros::ok())
  {
    key = getch();
    if (moveStatus.count(key) == 1)
    {
      // Grab the direction data
      x = moveStatus[key][0];
      y = moveStatus[key][1];
      th = moveStatus[key][2];
    }
    else
    {
      x = 0;
      y = 0;
      th = 0;
      if (key == '\x03')
      {
        Stop();
        std::cout << "exit!!" << std::endl;
        break;
      }
    }

    twist.linear.x = x * maxLinearVel_;
    twist.linear.y = y * maxLinearVel_;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = th * maxAngularVel_;

    ROS_INFO("twist: vx=%.2f, vy=%.2f vth=%.2f", twist.linear.x, twist.linear.y, twist.angular.z);

    cmdPub_.publish(twist);

    ros::spinOnce();
  }
}

int QKeyBoardTeleop::getch()
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

} // namespace IQR

int main(int argc, char **argv)
{
  ros::init(argc, argv, "keyboard_teleop");
  IQR::QKeyBoardTeleop keyboard_teleop;
  
  return 0;
}