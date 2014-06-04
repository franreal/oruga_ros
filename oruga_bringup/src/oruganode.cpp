#include <ros/ros.h>
#include <oruga_bringup/orugacomm.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "oruga");

  OrugaComm comm("oruga/data", "oruga/command", "/dev/ttyUSB0", 57600);
  ros::spin();

  return 0;
}

