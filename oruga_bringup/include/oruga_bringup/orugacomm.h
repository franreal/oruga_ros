#ifndef ORUGA_COMM
#define ORUGA_COMM

#include <oruga_msgs/ToOrugaData.h>
#include <serialcomm/serialcomm.h>
#include <serialcomm/criticalsection.h>
#include <ros/ros.h>

class OrugaComm: public SerialComm {
public:

  OrugaComm(std::string toOrugaTopic, std::string portName, int baudRate);
  ~OrugaComm();

  void writeToOrugaData();

private:

  void toOrugaCallback(const oruga_msgs::ToOrugaData::ConstPtr& msg) { toOrugaCs.set(*msg); }

  size_t buildFrameFromData(unsigned char *frame, oruga_msgs::ToOrugaData *data);

  void onRead(const unsigned char *data, unsigned int len) {}

  unsigned char *txBuffer;
  oruga_msgs::ToOrugaData toOrugaData;
  Critical<oruga_msgs::ToOrugaData> toOrugaCs;

  ros::NodeHandle nh;
  ros::Subscriber sub;

};

#endif  // ORUGA_COMM

