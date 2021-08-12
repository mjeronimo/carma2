#include "rclcpp/rclcpp.hpp"
#include "cav_msgs/msg/system_alert.hpp"
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <memory.h>
#include <unistd.h>

// Based off of: https://github.com/sukha-cn/turtlesim-ros2/blob/master/tutorials/teleop_turtle_key.cpp

// #define KEYCODE_R 0x43 
// #define KEYCODE_L 0x44
// #define KEYCODE_U 0x41
// #define KEYCODE_D 0x42
// #define KEYCODE_Q 0x71


#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36



class CarmaEventPub
{
public:
  CarmaEventPub(std::shared_ptr<rclcpp::Node> nh);
  void keyLoop();

private:

  
  std::shared_ptr<rclcpp::Node> nh_;
  double linear_, angular_, l_scale_, a_scale_;
  rclcpp::Publisher<cav_msgs::msg::SystemAlert>::SharedPtr event_pub_;
  
};

CarmaEventPub::CarmaEventPub(std::shared_ptr<rclcpp::Node> nh): nh_(nh)
{

  event_pub_ = nh_->create_publisher<cav_msgs::msg::SystemAlert>("/system_alert", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
//   ros::shutdown();
  rclcpp::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
//   ros::init(argc, argv, "teleop_turtle");
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("carma_event_pub");
 CarmaEventPub event_handler(node);

  signal(SIGINT,quit);

  event_handler.keyLoop();
  
  return(0);
}


void CarmaEventPub::keyLoop()
{
  char c;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle.");


  for(;;)
  {
    // get the next event from the keyboard  
    if(::read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_=angular_=0;
    cav_msgs::msg::SystemAlert new_msg;
    switch(c)
    {
      case KEYCODE_1:
        std::cout << "CAUTION" << std::endl;
        new_msg.type = cav_msgs::msg::SystemAlert::CAUTION;
        break;
      case KEYCODE_2:
        std::cout << "WARNING" << std::endl;
        new_msg.type = cav_msgs::msg::SystemAlert::WARNING;
        break;
      case KEYCODE_3:
        std::cout << "FATAL" << std::endl;
        new_msg.type = cav_msgs::msg::SystemAlert::FATAL;
        break;
      case KEYCODE_4:
        std::cout << "NOT_READY" << std::endl;
        new_msg.type = cav_msgs::msg::SystemAlert::NOT_READY;
        break;
      case KEYCODE_5:
        std::cout << "DRIVERS_READY" << std::endl;
        new_msg.type = cav_msgs::msg::SystemAlert::DRIVERS_READY;
        break;
      case KEYCODE_6:
        std::cout << "SHUTDOWN" << std::endl;
        new_msg.type = cav_msgs::msg::SystemAlert::SHUTDOWN;
        break;

        
    }
   
    new_msg.description = "dummy_message";
    event_pub_->publish(new_msg);
  }


  return;
}


