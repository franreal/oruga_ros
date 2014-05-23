#include <ros/ros.h>
#include <oruga_bringup/orugacomm.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "oruga");

  OrugaComm comm("oruga/command", "/dev/ttyUSB0", 57600);
  ros::Rate loop_rate(5);

  while(ros::ok()) {
    
    comm.writeToOrugaData();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

