#ifndef ORUGA_COMM
#define ORUGA_COMM

#include <oruga_msgs/OrugaData.h>
#include <serialcomm/serialcomm.h>
#include <ros/ros.h>

class OrugaComm: public SerialComm {
public:

  OrugaComm(std::string fromOrugaTopic, std::string toOrugaTopic, std::string portName, int baudRate);
  ~OrugaComm();

private:

  void toOrugaCallback(const oruga_msgs::OrugaData::ConstPtr& msg);

  size_t buildFrameFromData(unsigned char *frame, const oruga_msgs::OrugaData *data);
  void buildDataFromFrame (oruga_msgs::OrugaData *data, unsigned char* frame, size_t len);

  void onRead(const unsigned char *data, unsigned int len);

  unsigned char *txBuffer;

  int rxIndex;
  unsigned char *rxBuffer;
  enum { LOST, SYNC } rxStatus;
  
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;

};

#endif  // ORUGA_COMM
