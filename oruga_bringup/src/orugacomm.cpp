#include <oruga_bringup/orugacomm.h>

#define FRAME_HEADER 0xFE
#define FRAME_TAIL   0xFD
#define RX_FRAME_MAX_LENGTH 64
#define TX_FRAME_MAX_LENGTH 64

OrugaComm::OrugaComm(std::string fromOrugaTopic, std::string toOrugaTopic, std::string portName, int baudRate): 
rxIndex(0), rxStatus(LOST) {
  rxBuffer = new unsigned char[RX_FRAME_MAX_LENGTH];
  txBuffer = new unsigned char[TX_FRAME_MAX_LENGTH];
  memset(txBuffer, 0 , TX_FRAME_MAX_LENGTH);

  open(portName.c_str(), baudRate);
  startReadStream();

  pub = nh.advertise<oruga_msgs::OrugaData>(fromOrugaTopic, 1);
  sub = nh.subscribe(toOrugaTopic, 1, &OrugaComm::toOrugaCallback, this);
}

OrugaComm::~OrugaComm() {
  close();
  delete[] rxBuffer;
  delete[] txBuffer;
}

void OrugaComm::toOrugaCallback(const oruga_msgs::OrugaData::ConstPtr& msg) {
  size_t txFrameLength = buildFrameFromData(txBuffer, msg.get());
  write(txBuffer, txFrameLength);
//   for(unsigned int i = 0; i < txFrameLength; i++) {
//     write(&txBuffer[i], 1);
//     sleep(1);  // Debug!
//   }
}

size_t OrugaComm::buildFrameFromData(unsigned char *frame, const oruga_msgs::OrugaData *data) {
  size_t valueSize = data->value.size();
  frame[0] = FRAME_HEADER;
  frame[1] = data->code;
  for(size_t i = 0; i < valueSize; i++) {
    frame[i+2] = data->value[i];
  }
  frame[valueSize+2] = FRAME_TAIL;

  return (3+valueSize);
}

void OrugaComm::onRead(const unsigned char *data, unsigned int len) {

  for(unsigned int i = 0; i < len; i++) {

    unsigned char rx = data[i];
    switch(rxStatus) {

      case LOST:
        if(rx == FRAME_HEADER) {
          rxIndex = 0;
          rxBuffer[rxIndex++] = rx;
          rxStatus = SYNC;
        }
        break;

      case SYNC:
        rxBuffer[rxIndex++] = rx;
        if(rx == FRAME_TAIL) {
          oruga_msgs::OrugaData fromOrugaData;
          buildDataFromFrame(&fromOrugaData, rxBuffer, rxIndex);
          pub.publish(fromOrugaData);
          rxStatus = LOST;
        }
        if(rxIndex >= RX_FRAME_MAX_LENGTH) {
          rxStatus = LOST;
        }
        break;

      default:
        rxStatus = LOST;  // But it's somekind of error
    } 
  }  
}

void OrugaComm::buildDataFromFrame (oruga_msgs::OrugaData *data, unsigned char* frame, size_t len) {

  data->code = frame[1];
  for(size_t i = 2; i < len-1; i++) {
    data->value.push_back(frame[i]);
  }
//   printf("[");
//   for(unsigned int i = 0; i < len; i++) {
//     printf("%x\t", frame[i]);  // Debug!
//   }
//   printf("]\n");
}
