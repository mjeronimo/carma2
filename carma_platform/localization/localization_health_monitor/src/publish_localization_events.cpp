#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include <memory>

#include "cav_msgs/msg/localization_status_report.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time_source.hpp"

// Based off of: https://github.com/sukha-cn/turtlesim-ros2/blob/master/tutorials/teleop_turtle_key.cpp

#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36

int kfd = 0;
struct termios cooked, raw;

class LocalizationEventPub
{
public:
  explicit LocalizationEventPub(std::shared_ptr<rclcpp::Node> nh);
  void keyLoop();

private:
  std::shared_ptr<rclcpp::Node> nh_;
  rclcpp::Publisher<cav_msgs::msg::LocalizationStatusReport>::SharedPtr event_pub_;
  rclcpp::Clock::SharedPtr clock; 
  rclcpp::TimeSource ts;
};

LocalizationEventPub::LocalizationEventPub(std::shared_ptr<rclcpp::Node> nh)
: nh_(nh),ts(nh)
{
  event_pub_ = nh_->create_publisher<cav_msgs::msg::LocalizationStatusReport>("localization_status", 1);
  clock= std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ts.attachClock(clock);
}

void LocalizationEventPub::keyLoop()
{
  // Put the console into raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);

  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 1(UNINITIALIZED),2(INITIALIZING),3(OPERATIONAL),4(DEGRADED),5(DEGRADED_NO_LIDAR_FIX),6(AWAIT_MANUAL_INITIALIZATION) keys to issue system alerts");

  for (;; ) {
    // Get the next event from the keyboard
    char c;
    if (::read(kfd, &c, 1) < 0) {
      perror("read():");
      exit(-1);
    }

    cav_msgs::msg::LocalizationStatusReport new_msg;
    new_msg.header.stamp = clock->now();

    switch (c) {
      case KEYCODE_1:
        std::cout << "UNINITIALIZED" << std::endl;
        new_msg.status= cav_msgs::msg::LocalizationStatusReport::UNINITIALIZED;
        break;
      case KEYCODE_2:
        std::cout << "INITIALIZING" << std::endl;
        new_msg.status= cav_msgs::msg::LocalizationStatusReport::INITIALIZING;
        break;
      case KEYCODE_3:
        std::cout << "OPERATIONAL" << std::endl;
        new_msg.status= cav_msgs::msg::LocalizationStatusReport::OPERATIONAL;
        break;
      case KEYCODE_4:
        std::cout << "DEGRADED" << std::endl;
        new_msg.status= cav_msgs::msg::LocalizationStatusReport::DEGRADED;
        break;
      case KEYCODE_5:
        std::cout << "DEGRADED_NO_LIDAR_FIX" << std::endl;
        new_msg.status= cav_msgs::msg::LocalizationStatusReport::DEGRADED_NO_LIDAR_FIX;
        break;
      case KEYCODE_6:
        std::cout << "AWAIT_MANUAL_INITIALIZATION" << std::endl;
        new_msg.status= cav_msgs::msg::LocalizationStatusReport::AWAIT_MANUAL_INITIALIZATION;
        break;
      default:
        return;
    }

    event_pub_->publish(new_msg);
  }
}

void quit(int /*sig*/)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("localization_event_pub");
  LocalizationEventPub event_handler(node);
  signal(SIGINT, quit);
  event_handler.keyLoop();

  return 0;
}
