#include <ros/ros.h>
#include <oruga_msgs/OrugaData.h>
#include <oruga_msgs/operation_code.h>
#include <unistd.h>

class HeartBeat {
public:
  HeartBeat(std::string toOrugaTopic);
  void toggleLeds();
  
private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  oruga_msgs::OrugaData data;
};

HeartBeat::HeartBeat(std::string toOrugaTopic) {
  pub = nh.advertise<oruga_msgs::OrugaData>(toOrugaTopic, 1);
  data.code = operation_code::SWITCH_LEDS;
  data.value.push_back(0);
}

void HeartBeat::toggleLeds() {
  data.value[0] = !data.value[0];
  pub.publish(data);
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "heartbeat");
  HeartBeat heartbeat("oruga/command");
  double frequency = 1;  // Default value: 1Hz
  
  int c;  // Parse command-line options
  while ((c = getopt (argc, argv, "f:")) != -1) {
    switch (c) {
      case 'f':
        frequency = atof(optarg);
        break;
      case '?':
        if (optopt == 'c')
          fprintf (stderr, "Option -%c requires an argument.\n", optopt);
        else if (isprint (optopt))
          fprintf (stderr, "Unknown option `-%c'.\n", optopt);
        else
          fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
        return 1;
      default:
        abort();
    }
  }

  ros::Rate r(frequency);
  while (ros::ok()) {
    
    heartbeat.toggleLeds();
    
    ros::spinOnce();
    r.sleep();
  }
}
