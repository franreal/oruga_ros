#include <ros/ros.h>
#include <oruga_msgs/OrugaData.h>

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
  data.code = 0x22;
  data.value.push_back(0);
}

void HeartBeat::toggleLeds() {
  data.value[0] = !data.value[0];
  pub.publish(data);
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "heartbeat");
  HeartBeat heartbeat("oruga/command");

  ros::Rate r(2);  // 2Hz
  while (ros::ok()) {
    
    heartbeat.toggleLeds();
    
    ros::spinOnce();
    r.sleep();
  }
}
