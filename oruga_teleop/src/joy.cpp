#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <oruga_msgs/OrugaData.h>
#include <oruga_msgs/operation_code.h>

#define JOY_R_AXIS_INDEX 4
#define JOY_L_AXIS_INDEX 1

class JoyTeleop {
public:

  JoyTeleop(std::string joyTopic, std::string toOrugaTopic);
  ~JoyTeleop();

private:
  
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void buildDataFromJoy (oruga_msgs::OrugaData* data, const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh;

  ros::Publisher tooruga_pub;
  ros::Subscriber joy_sub;

};

JoyTeleop::JoyTeleop(std::string joyTopic, std::string toOrugaTopic) {
  joy_sub = nh.subscribe<sensor_msgs::Joy>(joyTopic, 10, &JoyTeleop::joyCallback, this);
  tooruga_pub = nh.advertise<oruga_msgs::OrugaData>(toOrugaTopic, 1);
}


JoyTeleop::~JoyTeleop() {}


void JoyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  oruga_msgs::OrugaData data;
  buildDataFromJoy(&data, joy);
  tooruga_pub.publish(data);
}


void JoyTeleop::buildDataFromJoy (oruga_msgs::OrugaData* data, const sensor_msgs::Joy::ConstPtr& joy) {
  data->code = operation_code::ABS_R_L_MOTORS;
  data->value.push_back( 125*(1+joy->axes[JOY_R_AXIS_INDEX]) );
  data->value.push_back( 125*(1+joy->axes[JOY_L_AXIS_INDEX]) );
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "joyteleop");

  JoyTeleop joyteleop("joy", "oruga/command");

  ros::spin();
}
