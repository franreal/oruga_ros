#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <ros/ros.h>
#include <oruga_msgs/ToOrugaData.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_P 0x20

class KeyTeleop {
public:
  KeyTeleop();
  void keyLoop();

private:

  ros::NodeHandle nh;
  ros::Publisher pub;

};

KeyTeleop::KeyTeleop() {

  pub = nh.advertise<oruga_msgs::ToOrugaData>("oruga/command", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig) {
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "keyteleop");
  KeyTeleop keyteleop;

  signal(SIGINT,quit);

  keyteleop.keyLoop();

  return(0);
}


void KeyTeleop::keyLoop() {
  char c;
  bool dirty=false;


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
  puts("Use arrow keys to move!");


  for(;;) {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0) {
      perror("read():");
      exit(-1);
    }

    //printf("value: 0x%02X\n", c);

    oruga_msgs::ToOrugaData data;

    switch(c) {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        data.code = 0x03;
        data.value.push_back(10);
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        data.code = 0x04;
        data.value.push_back(10);
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        data.code = 0x05;
        data.value.push_back(10);
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        data.code = 0x06;
        data.value.push_back(10);
        dirty = true;
        break;
      case KEYCODE_P:
        ROS_DEBUG("PANIC");
        data.code = 0xF4;
        data.value.push_back(125);
        data.value.push_back(125);
        dirty = true;
        break;
    }


    if(dirty ==true) {
      pub.publish(data);
      dirty=false;
    }
  }


  return;
}

